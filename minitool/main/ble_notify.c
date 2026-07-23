/*
 * ble_notify.c — Implementación del puente Gadgetbridge -> toasts.
 *
 * Se expone el Nordic UART Service (NUS), que es lo que espera el driver de
 * Bangle.js: una característica de escritura por donde el teléfono manda texto
 * y otra de notificación para responderle.
 *
 * Lo que llega viene troceado en escrituras de ~20 bytes (el MTU por defecto
 * de BLE), así que se acumula en un buffer hasta el salto de línea y recién
 * ahí se interpreta. Gadgetbridge antepone un 0x10 (Ctrl-P, "no eco" de
 * Espruino) y envuelve el JSON en GB(...): se salta el control y se recorta
 * entre el primer '{' y el último '}'.
 */
#include "ble_notify.h"
#include "ui_notify.h"
#include "bt_manager.h"
#include "weather_service.h"

#include "sdkconfig.h"
#include "esp_log.h"

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "ble_notify";

#if CONFIG_BT_NIMBLE_ENABLED

#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "cJSON.h"

/* Nordic UART Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * (los UUID de 128 bits van en orden inverso de bytes). */
static const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* RX: el teléfono escribe acá */
static const ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

/* TX: el reloj podría responder (Gadgetbridge se suscribe acá) */
static const ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

#define LINE_MAX 512

static uint16_t s_tx_handle = 0;
static char     s_line[LINE_MAX];
static int      s_line_len = 0;
static bool     s_linked = false;

/* Lo que está sonando en el teléfono (lo manda Gadgetbridge sin que se lo
 * pidan, al cambiar de tema o de estado). */
static char s_track[48] = "";
static char s_artist[48] = "";
static bool s_playing = false;
static bool s_music_valid = false;

/* ------------------------------ Interpretación --------------------------- */

/* Gadgetbridge manda la hora al conectarse: es una fuente de hora que no
 * depende de internet ni de que el SNTP resuelva. */
static void handle_set_time(const cJSON *root)
{
    const cJSON *jt = cJSON_GetObjectItem(root, "time");
    if (!cJSON_IsNumber(jt)) return;

    time_t t = (time_t)jt->valuedouble;
    if (t < 1600000000) return;          /* claramente inválida */

    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "Hora sincronizada desde el teléfono: %lld", (long long)t);
}

static void handle_notify(const cJSON *root)
{
    const cJSON *jsrc   = cJSON_GetObjectItem(root, "src");
    const cJSON *jtitle = cJSON_GetObjectItem(root, "title");
    const cJSON *jbody  = cJSON_GetObjectItem(root, "body");

    const char *src   = cJSON_IsString(jsrc)   ? jsrc->valuestring   : "Telefono";
    const char *title = cJSON_IsString(jtitle) ? jtitle->valuestring : "";
    const char *body  = cJSON_IsString(jbody)  ? jbody->valuestring  : "";

    /* El origen es la app (WhatsApp, Gmail...); el cuerpo, quién y qué. Si no
     * hay título, el mensaje va solo. */
    char msg[96];
    if (title[0] && body[0]) snprintf(msg, sizeof(msg), "%s: %s", title, body);
    else if (title[0])       snprintf(msg, sizeof(msg), "%s", title);
    else                     snprintf(msg, sizeof(msg), "%s", body);

    ui_notify_push(src, NOTIFY_INFO, msg);
}

static void handle_call(const cJSON *root)
{
    const cJSON *jcmd  = cJSON_GetObjectItem(root, "cmd");
    if (!cJSON_IsString(jcmd) || strcmp(jcmd->valuestring, "incoming") != 0) return;

    const cJSON *jname = cJSON_GetObjectItem(root, "name");
    const cJSON *jnum  = cJSON_GetObjectItem(root, "number");
    const char *who = cJSON_IsString(jname) && jname->valuestring[0] ? jname->valuestring
                    : (cJSON_IsString(jnum) ? jnum->valuestring : "Desconocido");

    /* Nivel ALERT: una llamada entrante merece el color fuerte. */
    ui_notify_push("Llamada", NOTIFY_ALERT, who);
}

/* Qué suena: Gadgetbridge manda "musicinfo" al cambiar de tema y
 * "musicstate" al pausar o reanudar. */
static void handle_music_info(const cJSON *root)
{
    const cJSON *jt = cJSON_GetObjectItem(root, "track");
    const cJSON *ja = cJSON_GetObjectItem(root, "artist");
    if (cJSON_IsString(jt)) strlcpy(s_track,  jt->valuestring, sizeof(s_track));
    if (cJSON_IsString(ja)) strlcpy(s_artist, ja->valuestring, sizeof(s_artist));
    s_music_valid = true;
}

static void handle_music_state(const cJSON *root)
{
    const cJSON *js = cJSON_GetObjectItem(root, "state");
    if (cJSON_IsString(js)) s_playing = (strcmp(js->valuestring, "play") == 0);
    s_music_valid = true;
}

/* Clima del teléfono. Vale más que el propio: el teléfono ya sabe dónde está y
 * tiene datos, así que el reloj deja de depender de servicios de internet que
 * pueden fallar o bloquearlo por límite de consultas.
 *
 * La temperatura viene en Kelvin en algunas versiones de Gadgetbridge y en °C
 * en otras; se distingue por el rango (nadie vive a 200 °C). */
static void handle_weather(const cJSON *root)
{
    const cJSON *jtemp = cJSON_GetObjectItem(root, "temp");
    const cJSON *jtxt  = cJSON_GetObjectItem(root, "txt");
    const cJSON *jloc  = cJSON_GetObjectItem(root, "loc");
    if (!cJSON_IsNumber(jtemp)) return;

    float temp = (float)jtemp->valuedouble;
    if (temp > 200.0f) temp -= 273.15f;

    weather_service_set_external(temp,
                                 cJSON_IsString(jtxt) ? jtxt->valuestring : "",
                                 cJSON_IsString(jloc) ? jloc->valuestring : "");
    ESP_LOGI(TAG, "Clima del teléfono: %.1f C", (double)temp);
}

static void handle_json(const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) return;

    const cJSON *jt = cJSON_GetObjectItem(root, "t");
    if (cJSON_IsString(jt)) {
        const char *t = jt->valuestring;
        if      (strcmp(t, "notify")     == 0) handle_notify(root);
        else if (strcmp(t, "call")       == 0) handle_call(root);
        else if (strcmp(t, "setTime")    == 0) handle_set_time(root);
        else if (strcmp(t, "musicinfo")  == 0) handle_music_info(root);
        else if (strcmp(t, "musicstate") == 0) handle_music_state(root);
        else if (strcmp(t, "weather")    == 0) handle_weather(root);
        s_linked = true;
    }
    cJSON_Delete(root);
}

/* Extrae el JSON de una línea "GB({...})" y lo procesa. */
static void handle_line(char *line)
{
    char *start = strchr(line, '{');
    char *end   = strrchr(line, '}');
    if (!start || !end || end < start) return;
    end[1] = '\0';
    handle_json(start);
}

/* Acumula lo que va llegando hasta tener una línea completa. */
static void feed(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n' || c == '\r') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                handle_line(s_line);
                s_line_len = 0;
            }
            continue;
        }
        if (s_line_len < LINE_MAX - 1) {
            s_line[s_line_len++] = c;
        } else {
            /* Línea desbordada: descartarla entera para no procesar basura. */
            s_line_len = 0;
            ESP_LOGW(TAG, "Línea demasiado larga, descartada");
        }
    }
}

/* --------------------------------- GATT ---------------------------------- */

static int nus_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[128];
        uint16_t out = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out) == 0) {
            feed(buf, (int)out);
        }
    }
    return 0;
}

static const struct ble_gatt_chr_def nus_chrs[] = {
    {
        .uuid       = &NUS_RX_UUID.u,
        .access_cb  = nus_access_cb,
        .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid       = &NUS_TX_UUID.u,
        .access_cb  = nus_access_cb,
        .flags      = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_tx_handle,
    },
    { 0 },
};

static const struct ble_gatt_svc_def nus_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &NUS_SVC_UUID.u,
        .characteristics = nus_chrs,
    },
    { 0 },
};

void ble_notify_register(void)
{
    int rc = ble_gatts_count_cfg(nus_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "count_cfg rc=%d", rc); return; }
    rc = ble_gatts_add_svcs(nus_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "add_svcs rc=%d", rc); return; }
    ESP_LOGI(TAG, "Servicio de notificaciones listo (Gadgetbridge / Bangle.js)");
}

bool ble_notify_linked(void) { return s_linked; }

/* ------------------------- Del reloj al teléfono ------------------------- */

/* Manda una línea JSON por la característica TX. Gadgetbridge está suscrito a
 * ella, así que le llega como notificación BLE. */
static bool send_json(const char *json)
{
    uint16_t conn = bt_manager_conn_handle();
    if (conn == 0xFFFF || s_tx_handle == 0) return false;

    char line[128];
    int n = snprintf(line, sizeof(line), "%s\n", json);
    if (n <= 0 || n >= (int)sizeof(line)) return false;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(line, (uint16_t)n);
    if (!om) return false;

    int rc = ble_gattc_notify_custom(conn, s_tx_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "No se pudo enviar (rc=%d)", rc);
        return false;
    }
    ESP_LOGI(TAG, "-> %s", json);
    return true;
}

bool ble_notify_music_cmd(const char *cmd)
{
    if (!cmd) return false;
    char json[64];
    snprintf(json, sizeof(json), "{\"t\":\"music\",\"n\":\"%s\"}", cmd);
    return send_json(json);
}

bool ble_notify_find_phone(bool on)
{
    char json[48];
    snprintf(json, sizeof(json), "{\"t\":\"findPhone\",\"n\":%s}", on ? "true" : "false");
    return send_json(json);
}

bool ble_notify_music_get(char *track, int track_size,
                          char *artist, int artist_size, bool *playing)
{
    if (!s_music_valid) return false;
    if (track)   strlcpy(track,  s_track,  track_size);
    if (artist)  strlcpy(artist, s_artist, artist_size);
    if (playing) *playing = s_playing;
    return true;
}

#else  /* sin NimBLE compilado */

void ble_notify_register(void) {}
bool ble_notify_linked(void) { return false; }
bool ble_notify_music_cmd(const char *cmd) { (void)cmd; return false; }
bool ble_notify_find_phone(bool on) { (void)on; return false; }
bool ble_notify_music_get(char *t, int ts, char *a, int as, bool *p)
{
    (void)t; (void)ts; (void)a; (void)as; (void)p; return false;
}

#endif
