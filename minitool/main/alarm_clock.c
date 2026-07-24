/*
 * alarm_clock.c — Implementación del despertador.
 *
 * El vigilante mira la hora cada 10 s. Para no repetir el disparo dentro del
 * mismo minuto —ni volver a sonar si el reloj se reinicia justo a esa hora— se
 * guarda de cada alarma el último día en que sonó: una alarma suena una vez
 * por día y se acabó.
 */
#include "alarm_clock.h"
#include "ui_power.h"
#include "ui_notify.h"

#include "lvgl.h"
#include "esp_log.h"
#include "nvs.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "alarm_clock";

#define TICK_MS     10000   /* basta para no perder un minuto */
#define FLASH_MS      450   /* ritmo del parpadeo */
#define FLASH_STEPS    40   /* ~18 s y se apaga sola */

#define ALARM_NS   "alarmclk"
#define ALARM_KEY  "alarms"

static alarm_t  s_alarms[ALARM_COUNT];
static int32_t  s_fired_day[ALARM_COUNT];   /* último día que sonó cada una */
static lv_timer_t *s_tick = NULL;

/* Overlay del aviso */
static lv_obj_t   *s_flash = NULL;
static lv_timer_t *s_flash_timer = NULL;
static int         s_flash_n = 0;

/* --------------------------------- NVS ---------------------------------- */

static void alarms_load(void)
{
    nvs_handle_t h;
    if (nvs_open(ALARM_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(s_alarms);
    nvs_get_blob(h, ALARM_KEY, s_alarms, &sz);
    nvs_close(h);
}

static void alarms_save(void)
{
    nvs_handle_t h;
    if (nvs_open(ALARM_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, ALARM_KEY, s_alarms, sizeof(s_alarms));
    nvs_commit(h);
    nvs_close(h);
}

/* ------------------------------- El aviso -------------------------------- */

void alarm_clock_dismiss(void)
{
    if (s_flash_timer) { lv_timer_del(s_flash_timer); s_flash_timer = NULL; }
    if (s_flash)       { lv_obj_del(s_flash);         s_flash = NULL; }
}

static void flash_click_cb(lv_event_t *e) { (void)e; alarm_clock_dismiss(); }

static void flash_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (!s_flash) return;
    s_flash_n++;
    bool on = s_flash_n % 2;
    lv_obj_set_style_bg_color(s_flash, on ? lv_color_hex(0x1E6FC8)
                                          : lv_color_hex(0x08172B), 0);
    if (s_flash_n >= FLASH_STEPS) alarm_clock_dismiss();
}

static void flash_show(const alarm_t *a)
{
    if (s_flash) return;

    /* Encender la pantalla es parte de despertar: el aviso es solo visual. */
    ui_power_wake();

    s_flash = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_flash);
    lv_obj_set_size(s_flash, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_flash, lv_color_hex(0x1E6FC8), 0);
    lv_obj_set_style_bg_opa(s_flash, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_flash, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_flash, flash_click_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *hora = lv_label_create(s_flash);
    lv_obj_set_style_text_font(hora, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(hora, lv_color_white(), 0);
    lv_label_set_text_fmt(hora, "%02d:%02d", a->hour, a->min);
    lv_obj_align(hora, LV_ALIGN_CENTER, 0, -14);

    lv_obj_t *sub = lv_label_create(s_flash);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(sub, lv_color_hex(0xCCE0F5), 0);
    lv_label_set_text(sub, "tocar para apagar");
    lv_obj_align(sub, LV_ALIGN_CENTER, 0, 34);

    s_flash_n = 0;
    s_flash_timer = lv_timer_create(flash_tick_cb, FLASH_MS, NULL);
}

/* -------------------------------- Vigilante ------------------------------ */

static int32_t today_id(const struct tm *tm)
{
    return (int32_t)(tm->tm_year * 366 + tm->tm_yday);
}

static void tick_cb(lv_timer_t *t)
{
    (void)t;

    time_t now = time(NULL);
    if (now < 1600000000) return;   /* sin hora en hora no hay despertador */

    struct tm tm;
    localtime_r(&now, &tm);
    int32_t day = today_id(&tm);

    for (int i = 0; i < ALARM_COUNT; i++) {
        if (!s_alarms[i].enabled) continue;
        if (s_alarms[i].hour != tm.tm_hour || s_alarms[i].min != tm.tm_min) continue;
        if (s_fired_day[i] == day) continue;   /* ya sonó hoy */

        s_fired_day[i] = day;
        ESP_LOGI(TAG, "Alarma %02d:%02d", s_alarms[i].hour, s_alarms[i].min);

        char msg[32];
        snprintf(msg, sizeof(msg), "%02d:%02d", s_alarms[i].hour, s_alarms[i].min);
        /* Nivel ALERT para que suene incluso en modo noche: una alarma que el
         * "no molestar" silencia no es una alarma. */
        ui_notify_push("Alarma", NOTIFY_ALERT, msg);
        flash_show(&s_alarms[i]);
        return;
    }
}

/* ------------------------------ API pública ------------------------------ */

void alarm_clock_init(void)
{
    if (s_tick) return;
    alarms_load();
    for (int i = 0; i < ALARM_COUNT; i++) s_fired_day[i] = -1;
    s_tick = lv_timer_create(tick_cb, TICK_MS, NULL);
    ESP_LOGI(TAG, "Despertador listo (%d alarmas activas)", alarm_clock_enabled_count());
}

bool alarm_clock_get(int i, alarm_t *out)
{
    if (i < 0 || i >= ALARM_COUNT || !out) return false;
    *out = s_alarms[i];
    return true;
}

void alarm_clock_set(int i, const alarm_t *a)
{
    if (i < 0 || i >= ALARM_COUNT || !a) return;
    s_alarms[i] = *a;
    /* Si se edita la hora, que pueda volver a sonar hoy mismo. */
    s_fired_day[i] = -1;
    alarms_save();
}

int alarm_clock_enabled_count(void)
{
    int n = 0;
    for (int i = 0; i < ALARM_COUNT; i++) if (s_alarms[i].enabled) n++;
    return n;
}
