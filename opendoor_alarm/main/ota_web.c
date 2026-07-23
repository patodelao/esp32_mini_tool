/*
 * ota_web.c — Implementación del servidor de actualización.
 */
#include "ota_web.h"

#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "ota_web";

#define CHUNK 1024

static httpd_handle_t    s_server = NULL;
static const char       *s_key = NULL;
static ota_web_notify_t  s_notify = NULL;

static void notify(const char *nivel, const char *msg)
{
    if (s_notify) s_notify(nivel, msg);
}

/* Compara la clave que viene en la query con la configurada. */
static bool key_ok(httpd_req_t *req)
{
    if (!s_key || !s_key[0]) return false;

    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;

    char got[64];
    if (httpd_query_key_value(query, "key", got, sizeof(got)) != ESP_OK) return false;

    return strcmp(got, s_key) == 0;
}

/* Página de estado: sirve para confirmar de un navegador que el nodo responde
 * y en qué ranura está corriendo. */
static esp_err_t root_get(httpd_req_t *req)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_app_desc_t desc;
    const char *ver = (esp_ota_get_partition_description(run, &desc) == ESP_OK)
                      ? desc.version : "?";

    char body[320];
    int n = snprintf(body, sizeof(body),
        "<html><body style='font-family:sans-serif'>"
        "<h3>Nodo refri</h3>"
        "<p>Ranura: <b>%s</b><br>Version: %s<br>Encendido: %llu min</p>"
        "<p>Actualizar:<br><code>curl -X POST --data-binary @firmware.bin "
        "\"http://IP/update?key=CLAVE\"</code></p>"
        "</body></html>",
        run ? run->label : "?", ver, esp_timer_get_time() / 60000000ULL);

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, body, n);
}

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

    ESP_LOGI(TAG, "Recibiendo firmware (%d bytes) hacia %s", req->content_len, target->label);
    notify("aviso", "Actualizando firmware por OTA");

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no se pudo empezar");
        return ESP_FAIL;
    }

    char buf[CHUNK];
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, remaining < CHUNK ? remaining : CHUNK);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;   /* reintentar */
        if (got <= 0) {
            ESP_LOGE(TAG, "Subida cortada (quedaban %d bytes)", remaining);
            esp_ota_abort(handle);
            notify("alarma", "OTA interrumpida; sigue el firmware viejo");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "subida cortada");
            return ESP_FAIL;
        }
        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "error escribiendo");
            return ESP_FAIL;
        }
        remaining -= got;
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

esp_err_t ota_web_start(const char *key, ota_web_notify_t notify_cb)
{
    if (s_server) return ESP_OK;

    s_key = key;
    s_notify = notify_cb;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    /* La subida es una sola petición larga; sin esto el socket vence a la
     * mitad en una red lenta. */
    cfg.recv_wait_timeout = 20;
    cfg.send_wait_timeout = 20;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    static const httpd_uri_t root = {
        .uri = "/", .method = HTTP_GET, .handler = root_get,
    };
    static const httpd_uri_t update = {
        .uri = "/update", .method = HTTP_POST, .handler = update_post,
    };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &update);

    ESP_LOGI(TAG, "OTA por Wi-Fi lista: POST /update?key=...");
    return ESP_OK;
}
