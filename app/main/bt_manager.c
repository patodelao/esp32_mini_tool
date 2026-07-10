/*
 * bt_manager.c — Advertising BLE con NimBLE, nombre persistido en NVS.
 * Si el firmware se compila sin CONFIG_BT_NIMBLE_ENABLED, quedan stubs que
 * reportan BT_STATE_UNSUPPORTED (la tool lo muestra en pantalla).
 */
#include "bt_manager.h"

#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "bt";

#define BT_NAME_DEFAULT "ESP32 MiniTool"
#define NVS_NAMESPACE "cfg"

static char s_name[BT_NAME_MAX + 1] = BT_NAME_DEFAULT;
static bool s_name_loaded = false;

static void load_name_once(void)
{
    if (s_name_loaded) return;
    s_name_loaded = true;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    size_t len = sizeof(s_name);
    nvs_get_str(nvs, "bt_name", s_name, &len);
    nvs_close(nvs);
}

static void save_name(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_str(nvs, "bt_name", s_name);
    nvs_commit(nvs);
    nvs_close(nvs);
}

void bt_manager_get_name(char *buf, size_t len)
{
    load_name_once();
    if (buf && len) strlcpy(buf, s_name, len);
}

#if CONFIG_BT_NIMBLE_ENABLED

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static bool s_stack_started = false; /* nimble_port corriendo */
static bool s_synced = false;        /* host sincronizado con el controlador */
static bool s_want_adv = false;      /* intención del usuario */
static bool s_peer_connected = false;
static uint8_t s_own_addr_type = 0;

static void start_advertising(void);

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_peer_connected = (event->connect.status == 0);
        ESP_LOGI(TAG, "Conexión BLE: %s", s_peer_connected ? "OK" : "falló");
        if (!s_peer_connected && s_want_adv) start_advertising();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        s_peer_connected = false;
        ESP_LOGI(TAG, "BLE desconectado");
        if (s_want_adv) start_advertising();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (s_want_adv) start_advertising();
        break;
    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    if (!s_synced) return; /* on_sync la lanzará al estar listo */

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)s_name;
    fields.name_len = strlen(s_name);
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &advp, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Visible como '%s'", s_name);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "No se pudo obtener dirección BLE (rc=%d)", rc);
        return;
    }
    s_synced = true;
    if (s_want_adv) start_advertising();
}

static void host_task(void *param)
{
    nimble_port_run(); /* retorna al llamar nimble_port_stop() */
    nimble_port_freertos_deinit();
}

static void stack_start_once(void)
{
    if (s_stack_started) return;
    load_name_once();
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_device_name_set(s_name);
    nimble_port_freertos_init(host_task);
    s_stack_started = true;
}

void bt_manager_start(void)
{
    s_want_adv = true;
    stack_start_once();
    start_advertising(); /* si aún no hay sync, on_sync la lanza */
}

void bt_manager_stop(void)
{
    s_want_adv = false;
    if (s_stack_started) {
        ble_gap_adv_stop();
    }
}

bt_state_t bt_manager_state(void)
{
    if (s_peer_connected) return BT_STATE_CONNECTED;
    if (s_want_adv && s_stack_started) return BT_STATE_ADVERTISING;
    return BT_STATE_OFF;
}

void bt_manager_set_name(const char *name)
{
    if (!name || !name[0]) return;
    strlcpy(s_name, name, sizeof(s_name));
    s_name_loaded = true;
    save_name();
    if (s_stack_started) {
        ble_svc_gap_device_name_set(s_name);
        /* Reanunciar con el nombre nuevo */
        if (s_want_adv) {
            ble_gap_adv_stop();
            start_advertising();
        }
    }
    ESP_LOGI(TAG, "Nombre BT: %s", s_name);
}

#else /* sin soporte BT compilado */

void bt_manager_start(void) { ESP_LOGW(TAG, "Firmware sin soporte BT"); }
void bt_manager_stop(void) {}
bt_state_t bt_manager_state(void) { return BT_STATE_UNSUPPORTED; }

void bt_manager_set_name(const char *name)
{
    if (!name || !name[0]) return;
    strlcpy(s_name, name, sizeof(s_name));
    s_name_loaded = true;
    save_name();
}

#endif /* CONFIG_BT_NIMBLE_ENABLED */
