/*
 * self_node.c — Implementación del auto-reporte del minitool.
 *
 * Un timer publica el estado y la telemetría cada minuto, mismo ritmo que usan
 * los nodos ESP. La IP y el "online" se publican también en cuanto hay
 * conexión, para no esperar el primer ciclo.
 */
#include "self_node.h"
#include "mqtt_hub.h"
#include "wifi_manager.h"
#include "fleet_service.h"
#include "bt_manager.h"   /* estado BLE para diagnóstico */

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"

#include <stdio.h>

static const char *TAG = "self_node";

#define TOPIC_IP     "labo/nodo/"   SELF_NODE_ID "/ip"
#define TOPIC_RSSI   "labo/sensor/" SELF_NODE_ID "/rssi"
#define TOPIC_HEAP   "labo/sensor/" SELF_NODE_ID "/heap"
#define TOPIC_UPTIME "labo/sensor/" SELF_NODE_ID "/uptime"
#define TOPIC_BLE    "labo/nodo/"   SELF_NODE_ID "/ble"   /* estado BLE (diagnóstico) */

#define PERIOD_US (60ULL * 1000000ULL)

static esp_timer_handle_t s_timer = NULL;
static bool s_announced = false;   /* ya se publicó el "online" de esta sesión */

/* IP actual como texto. false si todavía no hay. */
static bool get_ip(char *out, size_t len)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return false;
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == 0) return false;
    snprintf(out, len, IPSTR, IP2STR(&info.ip));
    return true;
}

static void publish_all(void)
{
    if (!mqtt_hub_connected()) {
        s_announced = false;   /* al reconectar hay que volver a anunciarse */
        return;
    }

    char buf[24];

    /* El "online" se republica en CADA ciclo, no una sola vez por sesión.
     *
     * Cuando el reloj se reinicia (un OTA, por ejemplo) el broker tarda unos
     * segundos en darse cuenta de que la conexión vieja murió y recién ahí
     * dispara su last-will retenido. Para entonces la sesión nueva ya publicó
     * su "online", así que el "offline" llega DESPUÉS y pisa el valor bueno:
     * el nodo queda marcado como caído estando vivo. Se vio en el propio panel
     * del reloj, que se mostraba a sí mismo offline recién actualizado.
     *
     * Insistir cada minuto lo repara solo. Son 13 bytes: no vale la pena
     * ninguna solución más fina. */
    mqtt_hub_publish(SELF_NODE_STATUS_TOPIC, "online", 1, true);

    /* Y además se marca a sí mismo, sin esperar el eco del broker: ese eco no
     * siempre vuelve, y el reloj terminaba mostrándose offline en su propia
     * tool Nodos estando conectado y publicando. */
    fleet_set_local(SELF_NODE_ID, true);

    if (!s_announced) {
        if (get_ip(buf, sizeof(buf))) mqtt_hub_publish(TOPIC_IP, buf, 1, true);
        s_announced = true;
        ESP_LOGI(TAG, "Anunciado como nodo '%s'", SELF_NODE_ID);
    }

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        snprintf(buf, sizeof(buf), "%d", ap.rssi);
        mqtt_hub_publish(TOPIC_RSSI, buf, 1, true);
    }

    snprintf(buf, sizeof(buf), "%.1f", esp_get_free_heap_size() / 1024.0f);
    mqtt_hub_publish(TOPIC_HEAP, buf, 1, true);

    snprintf(buf, sizeof(buf), "%llu", esp_timer_get_time() / 60000000ULL);
    mqtt_hub_publish(TOPIC_UPTIME, buf, 1, true);

    /* Estado BLE, para diagnosticar la conexión con Gadgetbridge sin cable serie:
     * "adv" = anunciando de verdad (si el celu igual no lo ve, es del lado del
     * teléfono); "off"/"errN" = no está anunciando (firmware). */
    mqtt_hub_publish(TOPIC_BLE, bt_manager_status_str(), 1, true);
}

static void tick_cb(void *arg) { (void)arg; publish_all(); }

void self_node_init(void)
{
    if (s_timer) return;
    mqtt_hub_init();

    const esp_timer_create_args_t args = { .callback = tick_cb, .name = "self_node" };
    if (esp_timer_create(&args, &s_timer) != ESP_OK) return;
    esp_timer_start_periodic(s_timer, PERIOD_US);

    /* Primer intento enseguida: si el MQTT todavía no conectó, el ciclo
     * siguiente lo resuelve. */
    publish_all();
    ESP_LOGI(TAG, "Auto-reporte cada %d s", (int)(PERIOD_US / 1000000ULL));
}
