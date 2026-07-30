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
#include "datalog.h"

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

/* Id apto para el DOM a partir del id del sensor ("pieza/temp" -> "pieza-temp").
 * Lo usa el refresco en vivo para encontrar cada celda por getElementById. */
static void dom_id(const char *id, char *out, int n)
{
    int j = 0;
    for (int i = 0; id[i] && j < n - 1; i++) {
        char c = id[i];
        bool alnum = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
        out[j++] = alnum ? c : '-';
    }
    out[j] = '\0';
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

/* ------------------------------ Gráficos SVG ------------------------------
 *
 * El panel mostraba un número por sensor. Un número solo no dice nada: 18.9 °C
 * puede ser una pieza estable o una que viene cayendo hace seis horas, y son
 * cosas distintas. La curva de las últimas 24 h contesta eso de un vistazo.
 *
 * Se dibuja en SVG generado acá mismo, sin librerías ni JavaScript: una
 * polilínea escalada al alto de la caja. Los datos salen de la historia horaria
 * que sensor_service ya mantiene en NVS, así que no hay que leer ni parsear el
 * CSV de la flash — que son cientos de kB y tardaría una eternidad.
 */
#define SPARK_W 300
#define SPARK_H  56

/* Devuelve false si no hay suficientes puntos para una curva. */
static bool sparkline(httpd_req_t *req, int idx, const char *color)
{
    float h[SENSOR_HIST_H];
    int hn = sensor_history_hourly(idx, h, SENSOR_HIST_H);
    if (hn < 2) return false;

    float mn = h[0], mx = h[0];
    for (int i = 1; i < hn; i++) {
        if (h[i] < mn) mn = h[i];
        if (h[i] > mx) mx = h[i];
    }
    /* Una serie plana (el suelo que no se movió en todo el día) dividiría por
     * cero y además quedaría pegada al borde; se le da aire artificial. */
    float span = mx - mn;
    if (span < 0.001f) { mn -= 1.0f; mx += 1.0f; span = mx - mn; }

    sendf(req, "<svg class='sp' viewBox='0 0 %d %d' preserveAspectRatio='none'>"
               "<polyline fill='none' stroke='%s' stroke-width='2' "
               "stroke-linejoin='round' points='", SPARK_W, SPARK_H, color);

    /* Los puntos se mandan de a poco: 24 pares no entran en el buffer de
     * sendf() y armar el string entero en la pila no vale la pena. */
    for (int i = 0; i < hn; i++) {
        int x = (hn == 1) ? 0 : (i * (SPARK_W - 4)) / (hn - 1) + 2;
        int y = SPARK_H - 4 - (int)(((h[i] - mn) / span) * (SPARK_H - 8));
        sendf(req, "%d,%d ", x, y);
    }
    send(req, "'/></svg>");
    return true;
}

/* --------------------------------- Página --------------------------------- */

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");

    send(req,
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='120'>"  /* fallback: si el JS falla igual se refresca */
        "<title>Home-lab</title><style>"
        "body{background:#0A0E12;color:#DDE6F0;font-family:system-ui,sans-serif;margin:0;padding:16px}"
        "h2{color:#8FA8C8;font-size:15px;font-weight:600;margin:24px 0 10px;"
        "text-transform:uppercase;letter-spacing:.08em}"
        "h3{color:#5A6B7A;font-size:12px;font-weight:600;margin:18px 0 8px;"
        "text-transform:uppercase;letter-spacing:.1em}"
        "table{width:100%;border-collapse:collapse}"
        "td{padding:9px 4px;border-bottom:1px solid #1A2733}"
        "td.v{text-align:right;font-variant-numeric:tabular-nums}"
        ".muted{color:#7F8C8D;font-size:13px}"
        ".ok{color:#35D07F}.off{color:#E74C3C}"
        "a{color:#35D07F}"
        /* Tarjeta por sensor: nombre arriba, valor grande, curva abajo. */
        ".g{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:10px}"
        ".c{background:#141C24;border:1px solid #1A2733;border-radius:12px;padding:12px 14px}"
        ".c .n{color:#8FA8C8;font-size:13px}"
        ".c .b{font-size:26px;font-variant-numeric:tabular-nums;margin:2px 0 6px}"
        ".c .u{font-size:14px;color:#7F8C8D;margin-left:3px}"
        ".sp{width:100%;height:56px;display:block}"
        ".f{display:flex;justify-content:space-between;font-size:12px;color:#5A6B7A;margin-top:4px}"
        ".hd{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:4px}"
        ".hd .t{color:#DDE6F0;font-size:18px;font-weight:600}"
        "</style></head><body>");

    /* Encabezado con estado del refresco en vivo (lo actualiza el JS del pie). */
    send(req,
        "<div class='hd'><span class='t'>Home-lab</span>"
        "<span id='live' style='font-size:12px;color:#35D07F'>&#9679; en vivo</span></div>");

    /* --- Sensores, agrupados por nodo ---
     *
     * Con 13 sensores de tres equipos una lista corrida no se lee. Agrupar por
     * nodo es como uno los piensa: "¿cómo está la pieza?", no "¿cómo está
     * pieza/temp?". */
    send(req, "<h2>Sensores</h2>");

    int n = sensor_count();
    if (n == 0) send(req, "<p class='muted'>sin sensores</p>");

    char done[8][24];        /* nodos ya impresos */
    int done_n = 0;

    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], val[16];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;

        char node[24];
        sensor_node_id(id, node, sizeof(node));

        bool seen = false;
        for (int d = 0; d < done_n; d++) if (strcmp(done[d], node) == 0) seen = true;
        if (seen) continue;
        if (done_n < (int)(sizeof(done) / sizeof(done[0]))) strlcpy(done[done_n++], node, 24);

        sendf(req, "<h3>%s</h3><div class='g'>", sensor_node_label(node));

        /* Segunda pasada: todos los sensores de ESTE nodo. */
        for (int j = 0; j < n; j++) {
            char jid[SENSOR_ID_MAX], jval[16], jname[40], jnode[24];
            uint32_t jage = 0;
            if (!sensor_get(j, jid, sizeof(jid), jval, sizeof(jval), &jage)) continue;
            sensor_node_id(jid, jnode, sizeof(jnode));
            if (strcmp(jnode, node) != 0) continue;

            sensor_friendly_name(jid, jname, sizeof(jname));
            const char *color = state_color(jid, jage);

            char jdom[48];
            dom_id(jid, jdom, sizeof(jdom));

            sendf(req, "<div class='c'><div class='n'>%s</div>"
                       "<div class='b'><span id='v-%s' style='color:%s'>%s</span>"
                       "<span class='u'>%s</span></div>",
                  jname, jdom, color, jval, sensor_unit(jid));

            if (!sparkline(req, j, color))
                send(req, "<div class='muted' style='height:56px'>sin historia todavia</div>");

            /* Pie de la tarjeta: récord del día y antigüedad del dato. */
            float rmn = 0, rmx = 0; bool rvalid = false;
            sensor_get_record(j, &rmn, &rmx, &rvalid);
            send(req, "<div class='f'><span>");
            if (rvalid) sendf(req, "hoy %.1f / %.1f", rmn, rmx);
            else        send(req, "24 h");
            sendf(req, "</span><span id='a-%s'>hace %us</span></div></div>", jdom, (unsigned)jage);
        }
        send(req, "</div>");
    }

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
                   "<td class='v %s' id='n-%s'>%s</td></tr>",
              nid, ip, online ? "ok" : "off", nid, online ? "online" : "offline");
    }
    if (fn == 0) send(req, "<tr><td class='muted'>sin nodos</td></tr>");
    send(req, "</table>");

    /* --- Cámara ---
     *
     * El nodo cam sirve su último frame en http://<ip-cam>/foto.jpg. Sacamos su
     * IP del fleet (retenida en labo/nodo/cam/ip) y embebemos la imagen; el
     * navegador la carga directo del cam (misma LAN / tailnet). El JS del pie la
     * refresca cada pocos segundos con un cache-buster, y si el cam no responde
     * muestra "sin conexión" en vez de una imagen rota. */
    {
        char cam_ip[16] = {0};
        int cn = fleet_count();
        for (int i = 0; i < cn; i++) {
            char cid[24];
            bool con = false;
            uint32_t cage = 0;
            if (!fleet_get(i, cid, sizeof(cid), &con, &cage)) continue;
            if (strcmp(cid, "cam") == 0) { fleet_get_ip(i, cam_ip, sizeof(cam_ip)); break; }
        }
        if (cam_ip[0]) {
            sendf(req, "<h2>Camara</h2>"
                       "<img id='cam' data-ip='%s' src='http://%s/foto.jpg' "
                       "style='width:100%%;max-width:480px;border-radius:12px;display:block'>"
                       "<p id='camoff' class='muted' style='display:none'>camara sin conexion</p>",
                  cam_ip, cam_ip);
        }
    }

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

    /* --- Registro histórico --- */
    if (datalog_ready()) {
        size_t kb = datalog_size() / 1024;
        if (kb > 0) {
            sendf(req, "<h2>Registro</h2><p class='muted'>%u kB guardados &middot; "
                       "una foto cada 30 min &middot; <a href='/csv'>descargar CSV</a></p>",
                  (unsigned)kb);
        } else {
            send(req, "<h2>Registro</h2><p class='muted'>todavia sin datos; "
                      "la primera foto se guarda a la media hora en punto</p>");
        }
    }

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

    /* --- Refresco en vivo ---
     *
     * En vez de recargar toda la página cada pocos segundos (parpadeo, datos,
     * scroll al tope), pedimos /data.json y actualizamos en el lugar solo los
     * valores y el estado de los nodos. Las curvas (horarias) siguen del render
     * inicial; el meta-refresh de 120 s queda de red de seguridad si esto falla.
     * Vanilla, sin librerías. */
    send(req,
        "<script>"
        "async function tick(){"
          "try{"
            "const r=await fetch('/data.json',{cache:'no-store'});"
            "const d=await r.json();"
            "for(const s of d.sensors){"
              "const v=document.getElementById('v-'+s.dom);"
              "if(v){v.textContent=s.v;v.style.color=s.c;}"
              "const a=document.getElementById('a-'+s.dom);"
              "if(a){a.textContent='hace '+s.age+'s';}"
            "}"
            "for(const n of d.nodes){"
              "const e=document.getElementById('n-'+n.id);"
              "if(e){e.textContent=n.on?'online':'offline';e.className='v '+(n.on?'ok':'off');}"
            "}"
            "const l=document.getElementById('live');"
            "if(l){l.textContent='\\u25CF en vivo';l.style.color='#35D07F';}"
          "}catch(e){"
            "const l=document.getElementById('live');"
            "if(l){l.textContent='\\u25CB sin conexion';l.style.color='#E74C3C';}"
          "}"
        "}"
        "setInterval(tick,4000);tick();"
        /* Cámara: precargar el frame nuevo y recién ahí cambiar el src (sin
           parpadeo); si falla la carga, mostrar 'sin conexion'. */
        "function camtick(){"
          "var c=document.getElementById('cam');if(!c)return;"
          "var ip=c.getAttribute('data-ip');"
          "var img=new Image();"
          "img.onload=function(){c.src=img.src;c.style.display='block';"
            "var o=document.getElementById('camoff');if(o)o.style.display='none';};"
          "img.onerror=function(){c.style.display='none';"
            "var o=document.getElementById('camoff');if(o)o.style.display='block';};"
          "img.src='http://'+ip+'/foto.jpg?t='+Date.now();"
        "}"
        "setInterval(camtick,8000);camtick();"
        "</script>");

    send(req, "</body></html>");
    httpd_resp_send_chunk(req, NULL, 0);   /* fin de la respuesta por trozos */
    return ESP_OK;
}

/* ------------------------------ Datos en vivo ----------------------------- */

/* JSON compacto que consume el JS de arriba: valores de sensores (con su color
 * de estado y antigüedad) y estado de los nodos. Deliberadamente NO trae las
 * curvas ni las alertas: cambian lento y el meta-refresh las cubre. */
static esp_err_t data_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");

    send(req, "{\"sensors\":[");
    int n = sensor_count();
    bool first = true;
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], val[16], dom[48];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;
        dom_id(id, dom, sizeof(dom));
        sendf(req, "%s{\"dom\":\"%s\",\"v\":\"%s\",\"c\":\"%s\",\"age\":%u}",
              first ? "" : ",", dom, val, state_color(id, age), (unsigned)age);
        first = false;
    }

    send(req, "],\"nodes\":[");
    int fn = fleet_count();
    first = true;
    for (int i = 0; i < fn; i++) {
        char nid[24];
        bool online = false;
        uint32_t age = 0;
        if (!fleet_get(i, nid, sizeof(nid), &online, &age)) continue;
        sendf(req, "%s{\"id\":\"%s\",\"on\":%s}",
              first ? "" : ",", nid, online ? "true" : "false");
        first = false;
    }
    send(req, "]}");

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* ------------------------------ Registro CSV ------------------------------ */

static bool csv_sink(void *ctx, const char *data, int len)
{
    /* Devolver false corta el volcado: el navegador cerró la descarga. */
    return httpd_resp_send_chunk((httpd_req_t *)ctx, data, len) == ESP_OK;
}

static esp_err_t csv_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=homelab.csv");

    if (!datalog_dump(csv_sink, req)) {
        httpd_resp_sendstr(req, "fecha,sensor,valor\n");   /* vacío, pero válido */
        return ESP_OK;
    }
    httpd_resp_send_chunk(req, NULL, 0);
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

    static const httpd_uri_t root   = { .uri = "/",          .method = HTTP_GET,  .handler = root_get };
    static const httpd_uri_t data   = { .uri = "/data.json", .method = HTTP_GET,  .handler = data_get };
    static const httpd_uri_t csv    = { .uri = "/csv",       .method = HTTP_GET,  .handler = csv_get };
    static const httpd_uri_t update = { .uri = "/update",    .method = HTTP_POST, .handler = update_post };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &data);
    httpd_register_uri_handler(s_server, &csv);
    httpd_register_uri_handler(s_server, &update);

    ESP_LOGI(TAG, "Panel web listo en http://<ip>/");
    return ESP_OK;
}
