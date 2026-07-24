#include "tool.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <time.h>

/* =========================================
 * 1. VARIABLES Y CONSTANTES
 * ========================================= */
#define CYCLE_DAYS_TARGET 13     /* Ej: 13 días de ciclo total */
#define SECONDS_IN_DAY 86400

static lv_obj_t *arc_progress = NULL;
static lv_obj_t *label_days = NULL;
static lv_obj_t *btn_reset = NULL;

/* Variable global de la herramienta para guardar el inicio */
static time_t start_time = 0; 

/* =========================================
 * 2. FUNCIONES DE MEMORIA NVS
 * ========================================= */
static void load_tracker_data(void) {
    nvs_handle_t my_handle;
    /* Abre el espacio "storage" en modo lectura/escritura */
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        /* Intenta leer la llave "start_time" */
        err = nvs_get_u32(my_handle, "start_time", (uint32_t *)&start_time);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            start_time = 0; /* Es la primera vez que se usa la herramienta */
        }
        nvs_close(my_handle); /* IMPORTANTE: Cerrar siempre */
    }
}

static void save_tracker_data(time_t new_time) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        /* Guarda el nuevo valor y fuerza la escritura física (commit) */
        nvs_set_u32(my_handle, "start_time", (uint32_t)new_time);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

/* =========================================
 * 3. LÓGICA DE INTERFAZ Y CALLBACKS
 * ========================================= */
static void update_ui_math(void) {
    if (start_time == 0) {
        lv_label_set_text(label_days, "No iniciado");
        lv_arc_set_value(arc_progress, 0);
        return;
    }

    /* Calcular tiempo transcurrido usando la hora actual del sistema */
    time_t now;
    time(&now);
    
    double elapsed_seconds = difftime(now, start_time);
    int elapsed_days = (int)(elapsed_seconds / SECONDS_IN_DAY);

    /* Actualizar textos */
    if (elapsed_days >= CYCLE_DAYS_TARGET) {
        lv_label_set_text(label_days, "¡Listo!");
        lv_obj_set_style_text_color(label_days, lv_color_hex(0x35D07F), 0); /* Verde */
        lv_arc_set_value(arc_progress, 100);
    } else {
        lv_label_set_text_fmt(label_days, "Día\n%d / %d", elapsed_days, CYCLE_DAYS_TARGET);
        
        /* Mapear días transcurridos al porcentaje del arco (0 - 100%) */
        int percent = (elapsed_days * 100) / CYCLE_DAYS_TARGET;
        lv_arc_set_value(arc_progress, percent);
    }
}

/* Callback del botón para reiniciar el ciclo */
static void btn_reset_cb(lv_event_t * e) {
    time_t now;
    time(&now);
    
    start_time = now;             /* Actualiza la variable local */
    save_tracker_data(now);       /* Persiste el dato en la Flash */
    update_ui_math();             /* Refresca la pantalla */
}

/* =========================================
 * 4. CICLO DE VIDA (Tool Interface)
 * ========================================= */
static void tracker_open(lv_obj_t *parent) {
    /* 1. Recuperar los datos guardados en la Flash ANTES de dibujar */
    load_tracker_data();

    /* 2. Dibujar el Arco de Progreso */
    arc_progress = lv_arc_create(parent);
    lv_obj_set_size(arc_progress, 200, 200);
    lv_arc_set_rotation(arc_progress, 270);
    lv_arc_set_bg_angles(arc_progress, 0, 360);
    lv_obj_remove_style(arc_progress, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc_progress, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(arc_progress);

    /* 3. Etiqueta central */
    label_days = lv_label_create(parent);
    lv_obj_center(label_days);
    lv_obj_set_style_text_align(label_days, LV_TEXT_ALIGN_CENTER, 0);

    /* 4. Botón de Planta/Reinicio en la parte inferior */
    btn_reset = lv_btn_create(parent);
    lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(btn_reset, btn_reset_cb, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t *btn_label = lv_label_create(btn_reset);
    lv_label_set_text(btn_label, "Iniciar Ciclo");

    /* 5. Calcular los días y pintar el estado actual */
    update_ui_math();
}

static void tracker_close(void) {
    /* No necesitamos liberar nada, LVGL destruye los widgets */
}

/* =========================================
 * 5. EXPORTACIÓN
 * ========================================= */
const tool_t tool_tracker = {
    .name = "Cycle Tracker",
    .icon = LV_SYMBOL_DRIVE,
    .accent = 0x9B59B6, /* Morado */
    .open = tracker_open,
    .close = tracker_close
};