/*
 * ota_web.c — Implementación del servidor de actualización.
 *
 * Calcado del nodo del refri (opendoor_alarm/main/ota_web.c): el mecanismo es
 * idéntico y ya está probado, incluidos los dos detalles que costó descubrir
 * (8 kB de pila para el handler y buffer estático). Lo único propio es el
 * nombre del nodo en la página de estado.
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

static const char *TAG = "ota_web";

/* Estático, no de pila: httpd atiende una petición por vez y 1 kB en la pila
 * de su task (4 kB por defecto) es justo lo que la hace desbordar a mitad de
 * la subida. */
#define CHUNK 1024
static char s_buf[CHUNK];

static httpd_handle_t    s_server = NULL;
static const char       *s_key = NULL;
static ota_web_notify_t  s_notify = NULL;

static void notify(const char *nivel, const char *msg)
{
    if (s_notify) s_notify(nivel, msg);
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

/* Página de estado: confirma desde un navegador que el nodo responde y en qué
 * ranura corre. */
static esp_err_t root_get(httpd_req_t *req)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_app_desc_t desc;
    const char *ver = (esp_ota_get_partition_description(run, &desc) == ESP_OK)
                      ? desc.version : "?";

    char body[420];
    int n = snprintf(body, sizeof(body),
        "<html><body style='font-family:sans-serif'>"
        "<h3>Nodo camara</h3>"
        "<p>Ranura: <b>%s</b><br>Version: %s<br>Encendido: %llu min</p>"
        "%s"
        "<p>Actualizar:<br><code>curl -X POST --data-binary @firmware.bin "
        "\"http://IP/update?key=CLAVE\"</code></p>"
        "</body></html>",
        run ? run->label : "?", ver, esp_timer_get_time() / 60000000ULL,
        camera_ready() ? "<p><a href='/foto.jpg'>Ver foto</a></p>"
                       : "<p><i>camara no disponible</i></p>");

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, body, n);
}

/* Una foto JPEG. El sensor ya entrega JPEG, así que el frame buffer se manda
 * tal cual, sin recomprimir. */
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

esp_err_t ota_web_start(const char *key, ota_web_notify_t notify_cb)
{
    if (s_server) return ESP_OK;

    s_key = key;
    s_notify = notify_cb;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    /* 4 kB (el valor por defecto) no alcanzan para recibir y escribir firmware:
     * la task del servidor desborda la pila a mitad de la subida y el nodo se
     * reinicia. Se nota como "connection reset" del lado del que sube. */
    cfg.stack_size = 8192;
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
    static const httpd_uri_t foto = {
        .uri = "/foto.jpg", .method = HTTP_GET, .handler = foto_get,
    };
    static const httpd_uri_t update = {
        .uri = "/update", .method = HTTP_POST, .handler = update_post,
    };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &foto);
    httpd_register_uri_handler(s_server, &update);

    ESP_LOGI(TAG, "OTA por Wi-Fi lista: POST /update?key=...");
    return ESP_OK;
}
