/*
 * tool_clock.c — Herramienta de tiempo con 3 mini-vistas conmutables por
 * pestañas inferiores:
 *   - Hora:  reloj digital + fecha (sincronizado por SNTP vía wifi_manager)
 *   - Crono: cronómetro play/pausa/reset (sigue corriendo en segundo plano)
 *   - Timer: temporizador con rollers de min/seg y alarma VISUAL al terminar
 *            (la placa no tiene parlante): la pantalla parpadea en rojo,
 *            incluso si el usuario salió de la herramienta.
 */
#include "tool.h"

#include <stdio.h>
#include <time.h>

#include "esp_timer.h"

/* ---------------------------------------------------------------- estado */

typedef enum { VIEW_CLOCK, VIEW_CHRONO, VIEW_TIMER } view_t;

static view_t s_view = VIEW_CLOCK;   /* vista activa (persiste entre aperturas) */
static lv_obj_t *s_content = NULL;   /* contenedor donde se dibuja la vista */
static lv_obj_t *s_tabs[3] = {0};    /* botones de pestaña */
static lv_timer_t *s_view_timer = NULL; /* timer de refresco de la vista activa */

/* Crono (persiste aunque se cierre la tool: solo aritmética de timestamps) */
static bool s_ch_running = false;
static int64_t s_ch_accum_us = 0;
static int64_t s_ch_started_us = 0;

/* Temporizador */
typedef enum { TMR_IDLE, TMR_RUNNING, TMR_PAUSED } tmr_state_t;
static tmr_state_t s_tmr_state = TMR_IDLE;
static int64_t s_tmr_target_us = 0; /* fin (válido en RUNNING) */
static int64_t s_tmr_remain_us = 0; /* restante (válido en PAUSED) */
static uint16_t s_sel_min = 5, s_sel_sec = 0; /* última selección de rollers */
static lv_timer_t *s_watch_timer = NULL; /* vigila expiración; vive GLOBAL */

/* Widgets de la vista activa (NULL si la vista no los usa) */
static lv_obj_t *s_main_label = NULL;
static lv_obj_t *s_sub_label = NULL;
static lv_obj_t *s_play_label = NULL;
static lv_obj_t *s_roller_min = NULL;
static lv_obj_t *s_roller_sec = NULL;

static void show_view(view_t v);

/* ------------------------------------------------- alarma visual global */

static lv_obj_t *s_flash = NULL;
static lv_timer_t *s_flash_timer = NULL;
static int s_flash_count = 0;

static void flash_stop(void)
{
    if (s_flash_timer) {
        lv_timer_del(s_flash_timer);
        s_flash_timer = NULL;
    }
    if (s_flash) {
        lv_obj_del(s_flash);
        s_flash = NULL;
    }
}

static void flash_click_cb(lv_event_t *e)
{
    (void)e;
    flash_stop();
}

static void flash_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (!s_flash) return;
    s_flash_count++;
    bool on = s_flash_count % 2;
    lv_obj_set_style_bg_color(s_flash, on ? lv_color_hex(0xC81E1E)
                                          : lv_color_hex(0x300000), 0);
    if (s_flash_count >= 24) flash_stop(); /* ~10 s y se apaga solo */
}

/* Pantalla completa parpadeante sobre TODO (lv_layer_top). Tocar la cierra. */
static void flash_alarm_show(void)
{
    if (s_flash) return;
    s_flash = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_flash);
    lv_obj_set_size(s_flash, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_flash, lv_color_hex(0xC81E1E), 0);
    lv_obj_set_style_bg_opa(s_flash, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_flash, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_flash, flash_click_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl = lv_label_create(s_flash);
    lv_label_set_text(lbl, "\xC2\xA1Tiempo!");
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_center(lbl);

    s_flash_count = 0;
    s_flash_timer = lv_timer_create(flash_tick_cb, 400, NULL);
}

/* Vigilante global: sobrevive al cierre de la tool para poder avisar. */
static void watch_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_tmr_state == TMR_RUNNING && esp_timer_get_time() >= s_tmr_target_us) {
        s_tmr_state = TMR_IDLE;
        flash_alarm_show();
        lv_timer_del(s_watch_timer);
        s_watch_timer = NULL;
        /* Si la vista Timer está abierta, reconstruirla al estado config */
        if (s_content && s_view == VIEW_TIMER) show_view(VIEW_TIMER);
    }
}

/* ------------------------------------------------------------ vista Hora */

static void clock_tick_cb(lv_timer_t *t)
{
    (void)t;
    time_t now;
    struct tm tm_info;
    time(&now);
    localtime_r(&now, &tm_info);

    char buf[16];
    strftime(buf, sizeof(buf), "%H:%M:%S", &tm_info);
    lv_label_set_text(s_main_label, buf);

    if (tm_info.tm_year < (2020 - 1900)) {
        lv_label_set_text(s_sub_label, "sin sincronizar");
    } else {
        strftime(buf, sizeof(buf), "%d/%m/%Y", &tm_info);
        lv_label_set_text(s_sub_label, buf);
    }
}

static void build_clock_view(void)
{
    s_main_label = lv_label_create(s_content);
    lv_obj_set_style_text_font(s_main_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_main_label, lv_color_white(), 0);
    lv_label_set_text(s_main_label, "--:--:--");
    lv_obj_align(s_main_label, LV_ALIGN_CENTER, 0, -20);

    s_sub_label = lv_label_create(s_content);
    lv_obj_set_style_text_font(s_sub_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_sub_label, lv_color_hex(0xBFE0FF), 0);
    lv_label_set_text(s_sub_label, "");
    lv_obj_align(s_sub_label, LV_ALIGN_CENTER, 0, 14);

    clock_tick_cb(NULL);
    s_view_timer = lv_timer_create(clock_tick_cb, 1000, NULL);
}

/* ----------------------------------------------------------- vista Crono */

static int64_t chrono_elapsed_us(void)
{
    return s_ch_accum_us + (s_ch_running ? esp_timer_get_time() - s_ch_started_us : 0);
}

static void chrono_render(void)
{
    int64_t ms = chrono_elapsed_us() / 1000;
    char buf[16];
    snprintf(buf, sizeof(buf), "%02d:%02d.%d",
             (int)(ms / 60000), (int)((ms / 1000) % 60), (int)((ms / 100) % 10));
    lv_label_set_text(s_main_label, buf);
}

static void chrono_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_ch_running) chrono_render();
}

static void chrono_play_cb(lv_event_t *e)
{
    (void)e;
    if (s_ch_running) {
        s_ch_accum_us += esp_timer_get_time() - s_ch_started_us;
        s_ch_running = false;
    } else {
        s_ch_started_us = esp_timer_get_time();
        s_ch_running = true;
    }
    lv_label_set_text(s_play_label, s_ch_running ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
}

static void chrono_reset_cb(lv_event_t *e)
{
    (void)e;
    s_ch_accum_us = 0;
    s_ch_started_us = esp_timer_get_time();
    chrono_render();
}

static lv_obj_t *round_btn(lv_obj_t *parent, lv_event_cb_t cb, lv_color_t color)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 48, 48);
    lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(btn, color, 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    return btn;
}

static void build_chrono_view(void)
{
    s_main_label = lv_label_create(s_content);
    lv_obj_set_style_text_font(s_main_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_main_label, lv_color_white(), 0);
    lv_obj_align(s_main_label, LV_ALIGN_CENTER, 0, -30);

    lv_obj_t *play = round_btn(s_content, chrono_play_cb, lv_color_hex(0x35D07F));
    lv_obj_align(play, LV_ALIGN_CENTER, -32, 25);
    s_play_label = lv_label_create(play);
    lv_label_set_text(s_play_label, s_ch_running ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
    lv_obj_center(s_play_label);

    lv_obj_t *reset = round_btn(s_content, chrono_reset_cb, lv_color_hex(0x33445A));
    lv_obj_align(reset, LV_ALIGN_CENTER, 32, 25);
    lv_obj_t *lbl = lv_label_create(reset);
    lv_label_set_text(lbl, LV_SYMBOL_REFRESH);
    lv_obj_center(lbl);

    chrono_render();
    s_view_timer = lv_timer_create(chrono_tick_cb, 100, NULL);
}

/* ----------------------------------------------------------- vista Timer */

static int64_t timer_remaining_us(void)
{
    if (s_tmr_state == TMR_RUNNING) {
        int64_t r = s_tmr_target_us - esp_timer_get_time();
        return r > 0 ? r : 0;
    }
    if (s_tmr_state == TMR_PAUSED) return s_tmr_remain_us;
    return 0;
}

static void timer_render(void)
{
    int64_t s = (timer_remaining_us() + 999999) / 1000000; /* redondear arriba */
    char buf[16];
    snprintf(buf, sizeof(buf), "%02d:%02d", (int)(s / 60), (int)(s % 60));
    lv_label_set_text(s_main_label, buf);
}

static void timer_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_main_label) timer_render();
}

static void ensure_watcher(void)
{
    if (!s_watch_timer) {
        s_watch_timer = lv_timer_create(watch_tick_cb, 250, NULL);
    }
}

static void timer_start_cb(lv_event_t *e)
{
    (void)e;
    s_sel_min = lv_roller_get_selected(s_roller_min);
    s_sel_sec = lv_roller_get_selected(s_roller_sec) * 5;
    int64_t total = ((int64_t)s_sel_min * 60 + s_sel_sec) * 1000000LL;
    if (total <= 0) return;
    s_tmr_target_us = esp_timer_get_time() + total;
    s_tmr_state = TMR_RUNNING;
    ensure_watcher();
    show_view(VIEW_TIMER); /* reconstruir en modo cuenta regresiva */
}

static void timer_pause_cb(lv_event_t *e)
{
    (void)e;
    if (s_tmr_state == TMR_RUNNING) {
        s_tmr_remain_us = timer_remaining_us();
        s_tmr_state = TMR_PAUSED;
    } else if (s_tmr_state == TMR_PAUSED) {
        s_tmr_target_us = esp_timer_get_time() + s_tmr_remain_us;
        s_tmr_state = TMR_RUNNING;
        ensure_watcher();
    }
    lv_label_set_text(s_play_label,
                      s_tmr_state == TMR_RUNNING ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
}

static void timer_stop_cb(lv_event_t *e)
{
    (void)e;
    s_tmr_state = TMR_IDLE;
    show_view(VIEW_TIMER); /* volver al selector */
}

static void build_timer_view(void)
{
    if (s_tmr_state == TMR_IDLE) {
        /* --- selector de duración --- */
        static char min_opts[300];
        static char sec_opts[64];
        if (min_opts[0] == '\0') {
            char *p = min_opts;
            for (int i = 0; i <= 60; i++)
                p += snprintf(p, sizeof(min_opts) - (p - min_opts), i ? "\n%d" : "%d", i);
            p = sec_opts;
            for (int i = 0; i <= 55; i += 5)
                p += snprintf(p, sizeof(sec_opts) - (p - sec_opts), i ? "\n%d" : "%d", i);
        }

        lv_obj_t *title = lv_label_create(s_content);
        lv_label_set_text(title, "min : seg");
        lv_obj_set_style_text_color(title, lv_color_hex(0xBFE0FF), 0);
        lv_obj_align(title, LV_ALIGN_CENTER, 0, -68);

        s_roller_min = lv_roller_create(s_content);
        lv_roller_set_options(s_roller_min, min_opts, LV_ROLLER_MODE_NORMAL);
        lv_roller_set_visible_row_count(s_roller_min, 3);
        lv_obj_set_width(s_roller_min, 62);
        lv_obj_align(s_roller_min, LV_ALIGN_CENTER, -40, -10);
        lv_roller_set_selected(s_roller_min, s_sel_min, LV_ANIM_OFF);

        s_roller_sec = lv_roller_create(s_content);
        lv_roller_set_options(s_roller_sec, sec_opts, LV_ROLLER_MODE_NORMAL);
        lv_roller_set_visible_row_count(s_roller_sec, 3);
        lv_obj_set_width(s_roller_sec, 62);
        lv_obj_align(s_roller_sec, LV_ALIGN_CENTER, 40, -10);
        lv_roller_set_selected(s_roller_sec, s_sel_sec / 5, LV_ANIM_OFF);

        lv_obj_t *start = lv_btn_create(s_content);
        lv_obj_set_size(start, 110, 36);
        lv_obj_align(start, LV_ALIGN_CENTER, 0, 55);
        lv_obj_set_style_radius(start, 18, 0);
        lv_obj_set_style_bg_color(start, lv_color_hex(0x35D07F), 0);
        lv_obj_add_event_cb(start, timer_start_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *lbl = lv_label_create(start);
        lv_label_set_text(lbl, "Iniciar");
        lv_obj_center(lbl);
    } else {
        /* --- cuenta regresiva --- */
        s_main_label = lv_label_create(s_content);
        lv_obj_set_style_text_font(s_main_label, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(s_main_label, lv_color_white(), 0);
        lv_obj_align(s_main_label, LV_ALIGN_CENTER, 0, -30);

        lv_obj_t *pause = round_btn(s_content, timer_pause_cb, lv_color_hex(0x35D07F));
        lv_obj_align(pause, LV_ALIGN_CENTER, -32, 25);
        s_play_label = lv_label_create(pause);
        lv_label_set_text(s_play_label,
                          s_tmr_state == TMR_RUNNING ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
        lv_obj_center(s_play_label);

        lv_obj_t *stop = round_btn(s_content, timer_stop_cb, lv_color_hex(0x5A3333));
        lv_obj_align(stop, LV_ALIGN_CENTER, 32, 25);
        lv_obj_t *lbl = lv_label_create(stop);
        lv_label_set_text(lbl, LV_SYMBOL_STOP);
        lv_obj_center(lbl);

        timer_render();
        s_view_timer = lv_timer_create(timer_tick_cb, 200, NULL);
    }
}

/* -------------------------------------------------------- infraestructura */

static void update_tab_styles(void)
{
    for (int i = 0; i < 3; i++) {
        if (!s_tabs[i]) continue;
        bool active = ((view_t)i == s_view);
        lv_obj_set_style_bg_color(s_tabs[i], active ? lv_color_hex(0x2E82C8)
                                                    : lv_color_hex(0x22303F), 0);
    }
}

static void show_view(view_t v)
{
    s_view = v;
    if (s_view_timer) {
        lv_timer_del(s_view_timer);
        s_view_timer = NULL;
    }
    s_main_label = s_sub_label = s_play_label = NULL;
    s_roller_min = s_roller_sec = NULL;
    lv_obj_clean(s_content);

    switch (v) {
    case VIEW_CLOCK:  build_clock_view();  break;
    case VIEW_CHRONO: build_chrono_view(); break;
    case VIEW_TIMER:  build_timer_view();  break;
    }
    update_tab_styles();
}

static void tab_click_cb(lv_event_t *e)
{
    show_view((view_t)(intptr_t)lv_event_get_user_data(e));
}

static void clock_open(lv_obj_t *parent)
{
    /* Contenido (deja libre la franja inferior para las pestañas) */
    s_content = lv_obj_create(parent);
    lv_obj_remove_style_all(s_content);
    lv_obj_set_size(s_content, 240, 186);
    lv_obj_align(s_content, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(s_content, LV_OBJ_FLAG_SCROLLABLE);

    /* Pestañas inferiores */
    static const char *names[3] = {"Hora", "Crono", "Timer"};
    for (int i = 0; i < 3; i++) {
        s_tabs[i] = lv_btn_create(parent);
        lv_obj_set_size(s_tabs[i], 54, 26);
        lv_obj_align(s_tabs[i], LV_ALIGN_BOTTOM_MID, (i - 1) * 58, -14);
        lv_obj_set_style_radius(s_tabs[i], 13, 0);
        lv_obj_add_event_cb(s_tabs[i], tab_click_cb, LV_EVENT_CLICKED,
                            (void *)(intptr_t)i);
        lv_obj_t *lbl = lv_label_create(s_tabs[i]);
        lv_label_set_text(lbl, names[i]);
        lv_obj_center(lbl);
    }

    show_view(s_view); /* restaurar la última vista usada */
}

static void clock_close(void)
{
    if (s_view_timer) {
        lv_timer_del(s_view_timer);
        s_view_timer = NULL;
    }
    /* OJO: s_watch_timer NO se borra: vigila el temporizador en background */
    s_content = NULL;
    s_main_label = s_sub_label = s_play_label = NULL;
    s_roller_min = s_roller_sec = NULL;
    for (int i = 0; i < 3; i++) s_tabs[i] = NULL;
}

const tool_t tool_clock = {
    .name = "Reloj",
    .icon = LV_SYMBOL_BELL,
    .accent = 0xFFB340,
    .open = clock_open,
    .close = clock_close,
};
