/*
 * ota_web.c — Servidor de control de la cámara + OTA.
 *
 * Dos servidores HTTP:
 *   - Puerto 80: panel de control, /foto.jpg, /stream-control (timelapse/ajustes)
 *     y el OTA (POST /update). Atiende una petición por vez.
 *   - Puerto 81: SOLO el stream MJPEG (/stream). Va aparte a propósito: un stream
 *     es una conexión que no termina, y si viviera en el server 80 bloquearía el
 *     OTA y el panel mientras alguien mira en vivo.
 *
 * Modos de captura (lo que se pidió):
 *   - Captura unitaria  -> /foto.jpg (un JPEG on-demand). Botón "Foto".
 *   - Grabación / vivo  -> /stream MJPEG en el puerto 81. Botón "En vivo".
 *   - Timelapse (periódica) -> el panel publica la config MQTT que ya consume el
 *     agente de la Pi (labo/config/cam/captura/{activo,intervalo}); la Pi guarda
 *     las fotos en su galería. El cam es la superficie de control; la Pi, el
 *     almacenamiento.
 *
 * El OTA está calcado del refri y es delicado (apagar la cámara antes de escribir
 * flash, 8 kB de pila, buffer estático): se preserva tal cual.
 */
#include "ota_web.h"

#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "ota_web";

/* Estático, no de pila: httpd atiende una petición por vez y 1 kB en la pila
 * de su task (4 kB por defecto) es justo lo que la hace desbordar a mitad de
 * la subida. */
#define CHUNK 1024
static char s_buf[CHUNK];

static httpd_handle_t     s_server = NULL;   /* control + OTA (puerto 80)  */
static httpd_handle_t     s_stream = NULL;   /* stream MJPEG (puerto 81)   */
static const char        *s_key = NULL;
static ota_web_notify_t   s_notify = NULL;
static ota_web_publish_t  s_publish = NULL;

/* Estado del timelapse (lo informa main.c desde la config retenida). */
static bool s_tl_active = false;
static int  s_tl_min = 10;

void ota_web_set_publish(ota_web_publish_t publish) { s_publish = publish; }
void ota_web_set_timelapse(bool active, int minutes)
{
    s_tl_active = active;
    if (minutes > 0) s_tl_min = minutes;
}

static void notify(const char *nivel, const char *msg)
{
    if (s_notify) s_notify(nivel, msg);
}

static void send(httpd_req_t *req, const char *s)
{
    httpd_resp_send_chunk(req, s, HTTPD_RESP_USE_STRLEN);
}

static bool key_ok(httpd_req_t *req)
{
    if (!s_key || !s_key[0]) return false;

    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;

    char got[64];
    if (httpd_query_key_value(query, "key", got, sizeof(got)) != ESP_OK) return false;

    return strcmp(got, s_key) == 0;
}

/* ------------------------------- Panel web -------------------------------- */

static esp_err_t root_get(httpd_req_t *req)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_app_desc_t desc;
    const char *ver = (esp_ota_get_partition_description(run, &desc) == ESP_OK)
                      ? desc.version : "?";

    httpd_resp_set_type(req, "text/html; charset=utf-8");

    send(req,
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Camara</title><style>"
        "body{background:#0A0E12;color:#DDE6F0;font-family:system-ui,sans-serif;margin:0;padding:16px;max-width:640px}"
        "h3{margin:0 0 10px}h4{color:#8FA8C8;margin:18px 0 8px;font-size:14px;text-transform:uppercase;letter-spacing:.06em}"
        "img{width:100%;border-radius:12px;display:block;background:#141C24;min-height:120px}"
        "button,select,input{font:inherit;padding:8px 14px;border-radius:10px;border:1px solid #2A3A48;"
        "background:#1A2733;color:#DDE6F0;margin:4px 4px 0 0}"
        "button{background:#33445A;cursor:pointer}button:active{background:#27384A}"
        ".muted{color:#7F8C8D;font-size:13px}code{background:#141C24;padding:2px 6px;border-radius:5px}"
        "</style></head><body>"
        "<h3>Camara</h3>"
        "<img id='cam' src='/foto.jpg' alt='camara'>"
        "<div>"
          "<button onclick='snap()'>Foto</button>"
          "<button onclick='live()'>En vivo</button>"
          "<button onclick='stop()'>Detener</button>"
        "</div>"
        "<h4>Timelapse</h4>"
        "<div class='muted'>Estado: <b id='tlst'>...</b> "
          "<span class='muted'>(las fotos las guarda la Pi)</span></div>"
        "<input id='tlmin' type='number' min='1' value='10' style='width:70px'> min "
        "<button onclick='tl(1)'>Iniciar</button>"
        "<button onclick='tl(0)'>Detener</button>"
        "<h4>Ajustes</h4>"
        "Resolucion "
        "<select id='res'>"
          "<option>QVGA</option><option>VGA</option><option selected>SVGA</option><option>XGA</option>"
        "</select> "
        "Calidad "
        "<select id='q'><option>10</option><option selected>12</option><option>16</option><option>20</option></select> "
        "<button onclick='setcam()'>Aplicar</button>");

    char sys[420];
    snprintf(sys, sizeof(sys),
        "<h4>Sistema</h4>"
        "<p class='muted'>Ranura %s &middot; version %s &middot; encendido %llu min</p>"
        "<p class='muted'>Actualizar:<br><code>curl -X POST --data-binary "
        "@build/esp32cam.bin \"http://IP/update?key=CLAVE\"</code></p>",
        run ? run->label : "?", ver, esp_timer_get_time() / 60000000ULL);
    send(req, sys);

    send(req,
        "<script>"
        "var ip=location.hostname;"
        "function snap(){document.getElementById('cam').src='/foto.jpg?t='+Date.now();}"
        "function live(){document.getElementById('cam').src='http://'+ip+':81/stream';}"
        "function stop(){snap();}"
        "function tl(o){var m=document.getElementById('tlmin').value;"
          "fetch('/tl?on='+o+'&min='+m).then(function(){setTimeout(st,400);});}"
        "function setcam(){var r=document.getElementById('res').value,q=document.getElementById('q').value;"
          "fetch('/set?res='+r+'&q='+q).then(function(){snap();});}"
        "function st(){fetch('/state').then(function(r){return r.json();}).then(function(d){"
          "document.getElementById('tlst').textContent=d.tl?('activo, cada '+d.min+' min'):'apagado';"
          "document.getElementById('tlmin').value=d.min;});}"
        "st();"
        "</script></body></html>");

    return httpd_resp_send_chunk(req, NULL, 0);
}

/* ------------------------ Captura unitaria (/foto.jpg) -------------------- */

static esp_err_t foto_get(httpd_req_t *req)
{
    if (!camera_ready()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "camara no disponible\n");
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "captura fallida");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "captura fallida");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=cam.jpg");
    esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);   /* SIEMPRE devolver el buffer, si no se agota */
    return r;
}

/* ------------------------- Grabación / vivo (/stream) --------------------- */

#define STREAM_BOUNDARY "frameboundary"
static esp_err_t stream_get(httpd_req_t *req)
{
    if (!camera_ready()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_sendstr(req, "camara no disponible\n");
    }

    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) break;                       /* cámara apagada (p.ej. por un OTA) */

        char hdr[80];
        int hl = snprintf(hdr, sizeof(hdr),
                          "\r\n--" STREAM_BOUNDARY "\r\n"
                          "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                          (unsigned)fb->len);
        esp_err_t r = httpd_resp_send_chunk(req, hdr, hl);
        if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);

        if (r != ESP_OK) break;               /* el cliente cerró la pestaña */
        vTaskDelay(pdMS_TO_TICKS(40));         /* ~20-25 fps tope y ceder CPU */
    }
    return ESP_OK;
}

/* --------------------- Control de timelapse (/tl, /state) ----------------- */

/* /tl?on=1&min=5 : publica la config retenida que consume el agente de la Pi. */
static esp_err_t tl_get(httpd_req_t *req)
{
    bool on = s_tl_active;
    int  min = s_tl_min;

    char query[48], v[12];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "on", v, sizeof(v)) == ESP_OK) on = (atoi(v) != 0);
        if (httpd_query_key_value(query, "min", v, sizeof(v)) == ESP_OK) {
            min = atoi(v);
            if (min < 1) min = 1;
        }
    }

    if (s_publish) {
        char mb[8];
        snprintf(mb, sizeof(mb), "%d", min);
        s_publish("labo/config/cam/captura/intervalo", mb, true);
        s_publish("labo/config/cam/captura/activo", on ? "1" : "0", true);
    }
    /* Reflejar de una, sin esperar el rebote de la config retenida. */
    ota_web_set_timelapse(on, min);

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t state_get(httpd_req_t *req)
{
    char b[48];
    int n = snprintf(b, sizeof(b), "{\"tl\":%d,\"min\":%d}", s_tl_active ? 1 : 0, s_tl_min);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, b, n);
}

/* ------------------------- Ajustes del sensor (/set) ---------------------- */

static esp_err_t set_get(httpd_req_t *req)
{
    sensor_t *s = esp_camera_sensor_get();
    char query[64], v[16];
    if (s && httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "res", v, sizeof(v)) == ESP_OK) {
            framesize_t fs = FRAMESIZE_SVGA;
            if      (strcmp(v, "QVGA") == 0) fs = FRAMESIZE_QVGA;
            else if (strcmp(v, "VGA")  == 0) fs = FRAMESIZE_VGA;
            else if (strcmp(v, "SVGA") == 0) fs = FRAMESIZE_SVGA;
            else if (strcmp(v, "XGA")  == 0) fs = FRAMESIZE_XGA;
            s->set_framesize(s, fs);
        }
        if (httpd_query_key_value(query, "q", v, sizeof(v)) == ESP_OK) {
            int q = atoi(v);
            if (q < 4) q = 4;
            if (q > 40) q = 40;
            s->set_quality(s, q);
        }
    }
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "ok");
}

/* ---------------------------------- OTA ----------------------------------- */

/* Reinicio diferido: hay que contestarle al curl antes de reiniciar, si no la
 * subida termina con "connection reset" aunque haya salido bien. */
static void restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

static esp_err_t update_post(httpd_req_t *req)
{
    if (!key_ok(req)) {
        ESP_LOGW(TAG, "Subida rechazada: clave incorrecta");
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "clave incorrecta");
        return ESP_FAIL;
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "sin particion OTA");
        return ESP_FAIL;
    }

    /* CLAVE en un nodo con cámara: apagar el sensor antes de escribir flash. Su
     * DMA I2S corre en segundo plano tocando PSRAM, y las escrituras a flash
     * deshabilitan la caché — la combinación cuelga el nodo a mitad de la
     * subida. Sin esto el OTA fallaba siempre a los ~15 s. */
    camera_stop();

    ESP_LOGI(TAG, "Recibiendo firmware (%d bytes) hacia %s", req->content_len, target->label);
    notify("aviso", "Actualizando firmware por OTA");

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no se pudo empezar");
        return ESP_FAIL;
    }

    int total = req->content_len;
    int remaining = total;
    int last_log = 0;
    while (remaining > 0) {
        int got = httpd_req_recv(req, s_buf, remaining < CHUNK ? remaining : CHUNK);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;   /* reintentar */
        if (got <= 0) {
            ESP_LOGE(TAG, "Subida cortada (quedaban %d bytes)", remaining);
            esp_ota_abort(handle);
            notify("alarma", "OTA interrumpida; sigue el firmware viejo");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "subida cortada");
            return ESP_FAIL;
        }
        err = esp_ota_write(handle, s_buf, got);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "error escribiendo");
            return ESP_FAIL;
        }
        remaining -= got;

        int done = total - remaining;
        if (done - last_log >= 128 * 1024) {
            last_log = done;
            ESP_LOGI(TAG, "OTA %d/%d kB", done / 1024, total / 1024);
        }
    }

    err = esp_ota_end(handle);   /* acá se valida el binario */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        notify("alarma", "Firmware invalido; no se aplico");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "firmware invalido");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no se pudo activar");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Firmware aplicado, reiniciando");
    notify("ok", "Firmware actualizado, reiniciando");
    httpd_resp_sendstr(req, "OK, reiniciando\n");

    xTaskCreate(restart_task, "ota_restart", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* --------------------------------- Arranque ------------------------------- */

esp_err_t ota_web_start(const char *key, ota_web_notify_t notify_cb)
{
    if (s_server) return ESP_OK;

    s_key = key;
    s_notify = notify_cb;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    /* 4 kB (por defecto) no alcanzan para recibir y escribir firmware: la task
     * del servidor desborda la pila a mitad de la subida y el nodo se reinicia. */
    cfg.stack_size = 8192;
    cfg.recv_wait_timeout = 20;
    cfg.send_wait_timeout = 20;
    cfg.max_uri_handlers = 8;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start (control): %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    static const httpd_uri_t u_root   = { .uri = "/",         .method = HTTP_GET,  .handler = root_get };
    static const httpd_uri_t u_foto   = { .uri = "/foto.jpg", .method = HTTP_GET,  .handler = foto_get };
    static const httpd_uri_t u_tl     = { .uri = "/tl",       .method = HTTP_GET,  .handler = tl_get };
    static const httpd_uri_t u_state  = { .uri = "/state",    .method = HTTP_GET,  .handler = state_get };
    static const httpd_uri_t u_set    = { .uri = "/set",      .method = HTTP_GET,  .handler = set_get };
    static const httpd_uri_t u_update = { .uri = "/update",   .method = HTTP_POST, .handler = update_post };
    httpd_register_uri_handler(s_server, &u_root);
    httpd_register_uri_handler(s_server, &u_foto);
    httpd_register_uri_handler(s_server, &u_tl);
    httpd_register_uri_handler(s_server, &u_state);
    httpd_register_uri_handler(s_server, &u_set);
    httpd_register_uri_handler(s_server, &u_update);

    /* Segundo servidor solo para el stream, en el puerto 81 (y con su propio
     * ctrl_port, si no chocan y el segundo no arranca). */
    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port   = 32769;
    scfg.lru_purge_enable = true;
    scfg.stack_size = 8192;
    if (httpd_start(&s_stream, &scfg) == ESP_OK) {
        static const httpd_uri_t u_stream = { .uri = "/stream", .method = HTTP_GET, .handler = stream_get };
        httpd_register_uri_handler(s_stream, &u_stream);
        ESP_LOGI(TAG, "Stream MJPEG en :81/stream");
    } else {
        ESP_LOGW(TAG, "No se pudo levantar el server de stream (:81)");
        s_stream = NULL;
    }

    ESP_LOGI(TAG, "Panel + OTA listos en :80");
    return ESP_OK;
}
