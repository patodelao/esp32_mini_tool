/*
 * menu_order.c — Implementación del orden de menú persistente en NVS.
 */
#include "menu_order.h"

#include "nvs.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "menu_order";

#define MAX_TOOLS     32
#define NVS_NS        "storage"
#define NVS_KEY_ORDER "menu_order"

static uint8_t s_order[MAX_TOOLS];
static int     s_count = 0;
static bool    s_loaded = false;

static void set_identity(void)
{
    for (int i = 0; i < s_count; i++) s_order[i] = (uint8_t)i;
}

/* Verifica que 'buf' sea una permutación válida de 0..s_count-1. */
static bool is_valid_permutation(const uint8_t *buf, int n)
{
    if (n != s_count) return false;
    bool seen[MAX_TOOLS] = { false };
    for (int i = 0; i < n; i++) {
        if (buf[i] >= s_count || seen[buf[i]]) return false;
        seen[buf[i]] = true;
    }
    return true;
}

static void save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_KEY_ORDER, s_order, s_count);
        nvs_commit(h);
        nvs_close(h);
    }
}

void menu_order_load(void)
{
    if (s_loaded) return;
    s_count = g_tools_count;
    if (s_count > MAX_TOOLS) s_count = MAX_TOOLS;
    set_identity();

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t buf[MAX_TOOLS];
        size_t len = sizeof(buf);
        if (nvs_get_blob(h, NVS_KEY_ORDER, buf, &len) == ESP_OK &&
            is_valid_permutation(buf, (int)len)) {
            memcpy(s_order, buf, len);
            ESP_LOGI(TAG, "Orden de menú cargado de NVS");
        }
        nvs_close(h);
    }
    s_loaded = true;
}

int menu_order_count(void) { return s_count; }

int menu_order_tool_index(int pos)
{
    if (pos < 0 || pos >= s_count) return 0;
    return s_order[pos];
}

const tool_t *menu_order_get(int pos)
{
    int idx = menu_order_tool_index(pos);
    if (idx < 0 || idx >= g_tools_count) return NULL;
    return g_tools[idx];
}

int menu_order_move(int pos, int dir)
{
    if (pos < 0 || pos >= s_count) return pos;
    int np = pos + dir;
    if (np < 0 || np >= s_count) return pos; /* borde: no se mueve */

    uint8_t tmp = s_order[pos];
    s_order[pos] = s_order[np];
    s_order[np] = tmp;
    save();
    return np;
}
