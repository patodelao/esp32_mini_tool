/*
 * web_ui.c — Implementación del panel web y del OTA del minitool.
 *
 * La página se envía por trozos (httpd_resp_send_chunk) en vez de armarla
 * entera en memoria: con 13 sensores, los nodos y 20 alertas se iría a varios
 * kB, y no hay razón para tenerlos todos juntos en RAM.
 *
 * Las lecturas de sensores/nodos se hacen desde la task del servidor, no desde
 * el hilo de LVGL. Son arreglos estáticos que solo crecen y se sobrescriben
 * campo a campo, así que lo peor que puede pasar es mostrar un valor de hace un
 * segundo — aceptable para una vista de solo lectura.
 */
#include "web_ui.h"
#include "sensor_service.h"
#include "sensor_alert.h"
#include "fleet_service.h"
#include "ui_notify.h"
#include "ui_theme.h"

#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

static const char *TAG = "web_ui";

/* Clave para actualizar por Wi-Fi. Va en la URL sobre HTTP plano: alcanza para
 * que nadie de la red lo reflashee por accidente, no contra alguien que espíe
 * el tráfico. */
#define WEB_OTA_KEY "minitool-ota"

#define CHUNK 1024
static char s_buf[CHUNK];     /* estático: 1 kB en la pila del httpd la desborda */

static httpd_handle_t s_server = NULL;

/* ------------------------------- Utilidades ------------------------------- */

static void send(httpd_req_t *req, const char *s) { httpd_resp_send_chunk(req, s, HTTPD_RESP_USE_STRLEN); }

static void sendf(httpd_req_t *req, const char *fmt, ...)
{
    static char line[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n > 0) httpd_resp_send_chunk(req, line, n);
}

static bool key_ok(httpd_req_t *req)
{
    char query[128], got[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, "key", got, sizeof(got)) != ESP_OK) return false;
    return strcmp(got, WEB_OTA_KEY) == 0;
}

/* Color CSS según el estado del sensor, el mismo criterio que en pantalla. */
static const char *state_color(const char *id, uint32_t age)
{
    if (age > sensor_alert_stale_limit(id)) return "#5A6B7A";
    switch (sensor_alert_state(id)) {
        case SENSOR_ALERT_LOW:
        case SENSOR_ALERT_HIGH: return "#E74C3C";
        default:                return "#DDE6F0";
    }
}

/* --------------------------------- Página --------------------------------- */

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");

    send(req,
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='15'>"   /* se refresca solo */
        "<title>Home-lab</title><style>"
        "body{background:#0A0E12;color:#DDE6F0;font-family:system-ui,sans-serif;margin:0;padding:16px}"
        "h2{color:#8FA8C8;font-size:15px;font-weight:600;margin:22px 0 8px;"
        "text-transform:uppercase;letter-spacing:.08em}"
        "table{width:100%;border-collapse:collapse}"
        "td{padding:9px 4px;border-bottom:1px solid #1A2733}"
        "td.v{text-align:right;font-variant-numeric:tabular-nums}"
        ".muted{color:#7F8C8D;font-size:13px}"
        ".ok{color:#35D07F}.off{color:#E74C3C}"
        "</style></head><body>");

    /* --- Sensores --- */
    send(req, "<h2>Sensores</h2><table>");
    int n = sensor_count();
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], val[16], name[40];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;
        sensor_friendly_name(id, name, sizeof(name));
        sendf(req, "<tr><td>%s</td><td class='v' style='color:%s'>%s %s</td>"
                   "<td class='v muted'>%us</td></tr>",
              name, state_color(id, age), val, sensor_unit(id), (unsigned)age);
    }
    if (n == 0) send(req, "<tr><td class='muted'>sin sensores</td></tr>");
    send(req, "</table>");

    /* --- Nodos --- */
    send(req, "<h2>Nodos</h2><table>");
    int fn = fleet_count();
    for (int i = 0; i < fn; i++) {
        char nid[24], ip[16];
        bool online = false;
        uint32_t age = 0;
        if (!fleet_get(i, nid, sizeof(nid), &online, &age)) continue;
        if (!fleet_get_ip(i, ip, sizeof(ip))) ip[0] = '\0';
        sendf(req, "<tr><td>%s <span class='muted'>%s</span></td>"
                   "<td class='v %s'>%s</td></tr>",
              nid, ip, online ? "ok" : "off", online ? "online" : "offline");
    }
    if (fn == 0) send(req, "<tr><td class='muted'>sin nodos</td></tr>");
    send(req, "</table>");

    /* --- Alertas --- */
    send(req, "<h2>Alertas</h2><table>");
    int an = ui_notify_history_count();
    for (int i = 0; i < an; i++) {
        notify_record_t r;
        if (!ui_notify_history_get(i, &r)) continue;
        char hora[8] = "--:--";
        if (r.ts > 0) {
            struct tm tm;
            localtime_r(&r.ts, &tm);
            snprintf(hora, sizeof(hora), "%02d:%02d", tm.tm_hour, tm.tm_min);
        }
        sendf(req, "<tr><td class='muted'>%s</td><td>%s</td>"
                   "<td class='v muted'>%s</td></tr>", hora, r.source, r.msg);
    }
    if (an == 0) send(req, "<tr><td class='muted'>sin alertas</td></tr>");
    send(req, "</table>");

    /* --- Pie: qué firmware corre y cómo actualizarlo --- */
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_app_desc_t desc;
    const char *ver = (esp_ota_get_partition_description(run, &desc) == ESP_OK) ? desc.version : "?";
    sendf(req, "<h2>Reloj</h2><p class='muted'>Ranura %s &middot; version %s &middot; "
               "encendido %llu min &middot; RAM %.1f kB</p>"
               "<p class='muted'>Actualizar:<br><code>curl -X POST --data-binary "
               "@build/spi_lcd_touch.bin \"http://IP/update?key=CLAVE\"</code></p>",
          run ? run->label : "?", ver,
          esp_timer_get_time() / 60000000ULL, esp_get_free_heap_size() / 1024.0f);

    send(req, "</body></html>");
    httpd_resp_send_chunk(req, NULL, 0);   /* fin de la respuesta por trozos */
    return ESP_OK;
}

/* ---------------------------------- OTA ----------------------------------- */

static void restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1200));   /* contestarle al curl antes de reiniciar */
    esp_restart();
}

static esp_err_t update_post(httpd_req_t *req)
{
    if (!key_ok(req)) {
        httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "clave incorrecta");
        return ESP_FAIL;
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "sin particion OTA");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Recibiendo firmware (%d bytes) hacia %s", req->content_len, target->label);
    ui_notify_push("Reloj", NOTIFY_WARNING, "Actualizando firmware");

    esp_ota_handle_t h = 0;
    if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no se pudo empezar");
        return ESP_FAIL;
    }

    int total = req->content_len, remaining = total, last_log = 0;
    while (remaining > 0) {
        int got = httpd_req_recv(req, s_buf, remaining < CHUNK ? remaining : CHUNK);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) {
            ESP_LOGE(TAG, "Subida cortada (quedaban %d bytes)", remaining);
            esp_ota_abort(h);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "subida cortada");
            return ESP_FAIL;
        }
        if (esp_ota_write(h, s_buf, got) != ESP_OK) {
            esp_ota_abort(h);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "error escribiendo");
            return ESP_FAIL;
        }
        remaining -= got;

        int done = total - remaining;
        if (done - last_log >= 256 * 1024) {
            last_log = done;
            ESP_LOGI(TAG, "OTA %d/%d kB", done / 1024, total / 1024);
        }
    }

    if (esp_ota_end(h) != ESP_OK) {        /* acá se valida el binario */
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "firmware invalido");
        return ESP_FAIL;
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no se pudo activar");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Firmware aplicado, reiniciando");
    httpd_resp_sendstr(req, "OK, reiniciando\n");
    xTaskCreate(restart_task, "ota_restart", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* ------------------------------ API pública ------------------------------- */

esp_err_t web_ui_start(void)
{
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    /* Los 4 kB por defecto no alcanzan para recibir y escribir firmware. */
    cfg.stack_size = 8192;
    cfg.recv_wait_timeout = 20;
    cfg.send_wait_timeout = 20;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    static const httpd_uri_t root   = { .uri = "/",       .method = HTTP_GET,  .handler = root_get };
    static const httpd_uri_t update = { .uri = "/update", .method = HTTP_POST, .handler = update_post };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &update);

    ESP_LOGI(TAG, "Panel web listo en http://<ip>/");
    return ESP_OK;
}
