/*
 * ui_notify.c — Implementación del sistema de notificaciones flotantes.
 *
 * Arquitectura (productor/consumidor desacoplado):
 *   - ui_notify_push()  → corre en cualquier tarea. Solo copia la notificación
 *                          a una cola de FreeRTOS. No toca LVGL.
 *   - drain_timer_cb()  → corre en el hilo de LVGL. Saca una notificación de la
 *                          cola y dibuja/anima el toast sobre lv_layer_top.
 *
 * Se muestra un toast a la vez; los siguientes esperan en la cola y aparecen
 * cuando el actual se cierra (auto-cierre por tiempo o toque del usuario).
 */
#include "ui_notify.h"
#include "ui_power.h"

#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"

#include <string.h>
#include <time.h>

static const char *TAG = "notify";

#define NOTIFY_QUEUE_LEN     8
#define NOTIFY_SRC_MAX       16
#define NOTIFY_MSG_MAX       80
#define NOTIFY_SHOW_MS       4200   /* tiempo en pantalla antes de auto-cerrar */
#define NOTIFY_WIDTH         194
#define NOTIFY_TOP_Y         22     /* margen desde el borde superior */
#define NOTIFY_DRAIN_MS      150

typedef struct {
    char           source[NOTIFY_SRC_MAX];
    char           msg[NOTIFY_MSG_MAX];
    notify_level_t level;
} notify_item_t;

static QueueHandle_t s_queue = NULL;
static lv_timer_t   *s_drain_timer = NULL;

/* Estado del toast actualmente visible (solo lo toca el hilo de LVGL) */
static lv_obj_t   *s_toast = NULL;
static lv_timer_t *s_hide_timer = NULL;

/* Historial (ring buffer). Ver la sección "Historial" más abajo. */
static notify_record_t   s_hist[NOTIFY_HIST];
static int               s_hist_n = 0;     /* válidos, tope NOTIFY_HIST */
static int               s_hist_head = 0;  /* dónde va el próximo */
static SemaphoreHandle_t s_hist_lock = NULL;
static volatile bool     s_hist_dirty = false;  /* hay cambios sin guardar */
static uint32_t          s_hist_saved_ms = 0;   /* último guardado */

#define HIST_NS       "notifhist"
#define HIST_KEY      "hist"
#define HIST_SAVE_MS  10000   /* no escribir flash más seguido que esto */

/* Definidas en la sección "Historial", más abajo; se usan antes. */
static void history_load(void);
static void history_save(void);

/* --- Estilo por nivel ------------------------------------------------------ */

static const char *level_symbol(notify_level_t lv)
{
    switch (lv) {
        case NOTIFY_SUCCESS: return LV_SYMBOL_OK;
        case NOTIFY_WARNING: return LV_SYMBOL_WARNING;
        case NOTIFY_ALERT:   return LV_SYMBOL_BELL;
        case NOTIFY_INFO:
        default:             return LV_SYMBOL_LIST;
    }
}

static lv_color_t level_color(notify_level_t lv)
{
    switch (lv) {
        case NOTIFY_SUCCESS: return lv_color_hex(0x2ECC71);
        case NOTIFY_WARNING: return lv_color_hex(0xF1C40F);
        case NOTIFY_ALERT:   return lv_color_hex(0xE74C3C);
        case NOTIFY_INFO:
        default:             return lv_color_hex(0x3498DB);
    }
}

/* --- Cierre / animación ---------------------------------------------------- */

static void toast_deleted_cb(lv_anim_t *a)
{
    lv_obj_t *obj = (lv_obj_t *)a->var;
    if (obj) lv_obj_del(obj);
    if (obj == s_toast) s_toast = NULL;
}

static void close_toast(void)
{
    if (s_hide_timer) {
        lv_timer_del(s_hide_timer);
        s_hide_timer = NULL;
    }
    if (!s_toast) return;

    /* Deslizar hacia arriba y borrar al terminar */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_toast);
    lv_anim_set_values(&a, lv_obj_get_y(s_toast), -120);
    lv_anim_set_time(&a, 220);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_set_ready_cb(&a, toast_deleted_cb);
    /* s_toast se pone a NULL en el ready_cb; mientras tanto marcamos que ya
       no está "activo" para permitir el siguiente de la cola. */
    lv_obj_t *going = s_toast;
    s_toast = NULL;
    lv_anim_set_var(&a, going);
    lv_anim_start(&a);
}

static void hide_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_hide_timer = NULL;   /* este timer es one-shot */
    close_toast();
}

static void toast_click_cb(lv_event_t *e)
{
    (void)e;
    close_toast();
}

/* --- Construcción del toast (hilo de LVGL) --------------------------------- */

static void show_toast(const notify_item_t *it)
{
    /* Un aviso con la pantalla apagada no sirve de nada: encenderla es parte
     * de notificar. También reinicia la inactividad, así queda tiempo para
     * leerlo antes de que se vuelva a dormir. */
    ui_power_wake();

    lv_color_t accent = level_color(it->level);

    lv_obj_t *card = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(card);
    lv_obj_set_width(card, NOTIFY_WIDTH);
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(card, lv_color_hex(0x11202E), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_pad_all(card, 10, 0);
    lv_obj_set_style_pad_left(card, 14, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, accent, 0);
    lv_obj_set_style_shadow_width(card, 12, 0);
    lv_obj_set_style_shadow_color(card, lv_color_black(), 0);
    lv_obj_set_style_shadow_opa(card, LV_OPA_40, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(card, toast_click_cb, LV_EVENT_CLICKED, NULL);

    /* Layout en fila: icono + (origen / mensaje) */
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(card, 10, 0);

    lv_obj_t *icon = lv_label_create(card);
    lv_label_set_text(icon, level_symbol(it->level));
    lv_obj_set_style_text_color(icon, accent, 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);

    lv_obj_t *col = lv_obj_create(card);
    lv_obj_remove_style_all(col);
    lv_obj_set_width(col, LV_PCT(100));
    lv_obj_set_height(col, LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(col, 1);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);

    if (it->source[0]) {
        lv_obj_t *src = lv_label_create(col);
        lv_label_set_text(src, it->source);
        lv_obj_set_style_text_color(src, accent, 0);
        lv_obj_set_style_text_font(src, &lv_font_montserrat_14, 0);
    }

    lv_obj_t *msg = lv_label_create(col);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg, LV_PCT(100));
    lv_label_set_text(msg, it->msg);
    lv_obj_set_style_text_color(msg, lv_color_white(), 0);
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_16, 0);

    lv_obj_align(card, LV_ALIGN_TOP_MID, 0, NOTIFY_TOP_Y);
    s_toast = card;

    /* Animación de entrada: deslizar desde arriba */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, card);
    lv_anim_set_values(&a, -120, NOTIFY_TOP_Y);
    lv_anim_set_time(&a, 260);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);

    /* Auto-cierre */
    s_hide_timer = lv_timer_create(hide_timer_cb, NOTIFY_SHOW_MS, NULL);
    lv_timer_set_repeat_count(s_hide_timer, 1);

    ESP_LOGI(TAG, "Toast [%s] %s", it->source, it->msg);
}

/* --- Drenado de la cola (hilo de LVGL) ------------------------------------- */

static void drain_timer_cb(lv_timer_t *t)
{
    (void)t;

    /* Persistir el historial acá (hilo de LVGL) y con un mínimo entre
     * escrituras: una ráfaga de alertas no debe traducirse en una escritura
     * de flash por cada una. */
    if (s_hist_dirty) {
        uint32_t now = lv_tick_get();
        if (now - s_hist_saved_ms >= HIST_SAVE_MS) {
            s_hist_dirty = false;
            s_hist_saved_ms = now;
            history_save();
        }
    }

    if (s_toast) return;               /* espera a que el actual se cierre */
    if (!s_queue) return;

    notify_item_t it;
    if (xQueueReceive(s_queue, &it, 0) == pdTRUE) {
        show_toast(&it);
    }
}

/* --- API pública ----------------------------------------------------------- */

void ui_notify_init(void)
{
    if (s_queue) return; /* idempotente */
    s_hist_lock = xSemaphoreCreateMutex();
    history_load();      /* el historial sobrevive al reinicio */
    s_queue = xQueueCreate(NOTIFY_QUEUE_LEN, sizeof(notify_item_t));
    if (!s_queue) {
        ESP_LOGE(TAG, "No se pudo crear la cola de notificaciones");
        return;
    }
    s_drain_timer = lv_timer_create(drain_timer_cb, NOTIFY_DRAIN_MS, NULL);
    ESP_LOGI(TAG, "Sistema de notificaciones listo");
}

/* --- Historial -------------------------------------------------------------
 *
 * Ring buffer protegido con un mutex: ui_notify_push() puede venir de la tarea
 * MQTT o de un timer, y la tool Alertas lee desde el hilo de LVGL.
 */
/* El blob guarda el ring tal cual, con su cabeza y su cuenta. */
typedef struct {
    notify_record_t rec[NOTIFY_HIST];
    int16_t         n, head;
} hist_blob_t;

static void history_load(void)
{
    nvs_handle_t h;
    if (nvs_open(HIST_NS, NVS_READONLY, &h) != ESP_OK) return;

    static hist_blob_t blob;   /* ~2 kB: static para no cargar el stack */
    size_t sz = sizeof(blob);
    if (nvs_get_blob(h, HIST_KEY, &blob, &sz) == ESP_OK && sz == sizeof(blob)) {
        memcpy(s_hist, blob.rec, sizeof(s_hist));
        s_hist_n    = blob.n    > NOTIFY_HIST ? NOTIFY_HIST : blob.n;
        s_hist_head = blob.head % NOTIFY_HIST;
    }
    nvs_close(h);
}

/* Se llama desde el hilo de LVGL (drain timer), no desde el productor: así una
 * ráfaga de alertas no dispara una escritura de flash por cada una. */
static void history_save(void)
{
    if (!s_hist_lock) return;
    static hist_blob_t blob;
    if (xSemaphoreTake(s_hist_lock, pdMS_TO_TICKS(50)) != pdTRUE) return;
    memcpy(blob.rec, s_hist, sizeof(blob.rec));
    blob.n    = (int16_t)s_hist_n;
    blob.head = (int16_t)s_hist_head;
    xSemaphoreGive(s_hist_lock);

    nvs_handle_t h;
    if (nvs_open(HIST_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, HIST_KEY, &blob, sizeof(blob));
    nvs_commit(h);
    nvs_close(h);
}

static void history_add(const notify_item_t *it)
{
    if (!s_hist_lock) return;
    if (xSemaphoreTake(s_hist_lock, pdMS_TO_TICKS(50)) != pdTRUE) return;

    notify_record_t *r = &s_hist[s_hist_head];
    strlcpy(r->source, it->source, sizeof(r->source));
    strlcpy(r->msg,    it->msg,    sizeof(r->msg));
    r->level = it->level;
    time_t now = time(NULL);
    r->ts = (now > 1600000000) ? now : 0;   /* 0 = reloj sin sincronizar */

    s_hist_head = (s_hist_head + 1) % NOTIFY_HIST;
    if (s_hist_n < NOTIFY_HIST) s_hist_n++;

    xSemaphoreGive(s_hist_lock);
    s_hist_dirty = true;   /* el drain timer lo persiste */
}

int ui_notify_history_count(void)
{
    return s_hist_n;
}

bool ui_notify_history_get(int i, notify_record_t *out)
{
    if (!out || !s_hist_lock || i < 0 || i >= s_hist_n) return false;
    if (xSemaphoreTake(s_hist_lock, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    /* i = 0 es el más reciente: el anterior a head. */
    int idx = (s_hist_head - 1 - i + 2 * NOTIFY_HIST) % NOTIFY_HIST;
    *out = s_hist[idx];
    xSemaphoreGive(s_hist_lock);
    return true;
}

void ui_notify_history_clear(void)
{
    if (!s_hist_lock) return;
    if (xSemaphoreTake(s_hist_lock, pdMS_TO_TICKS(50)) != pdTRUE) return;
    s_hist_n = 0;
    s_hist_head = 0;
    memset(s_hist, 0, sizeof(s_hist));
    xSemaphoreGive(s_hist_lock);
    history_save();   /* el borrado se persiste ya: es una acción del usuario */
    s_hist_dirty = false;
}

void ui_notify_push(const char *source, notify_level_t level, const char *msg)
{
    if (!s_queue) {
        ESP_LOGW(TAG, "push antes de init; descartada");
        return;
    }
    notify_item_t it = { .level = level };
    if (source) strlcpy(it.source, source, sizeof(it.source));
    if (msg)    strlcpy(it.msg,    msg,    sizeof(it.msg));

    /* Al historial va siempre, aunque el toast se descarte por cola llena. */
    history_add(&it);

    /* Modo noche: no interrumpir. La notificación queda registrada (se lee
     * después en la tool Alertas) pero no se dibuja ni enciende la pantalla.
     * Las críticas pasan igual: para eso son críticas. */
    if (level != NOTIFY_ALERT && ui_power_quiet_now()) {
        ESP_LOGI(TAG, "En silencio: [%s] %s va solo al historial", it.source, it.msg);
        return;
    }

    /* No bloquear al productor: si la cola está llena, descartar la más vieja */
    if (xQueueSend(s_queue, &it, 0) != pdTRUE) {
        notify_item_t drop;
        xQueueReceive(s_queue, &drop, 0);
        xQueueSend(s_queue, &it, 0);
    }
}
