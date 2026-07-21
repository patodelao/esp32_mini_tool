/* =========================================
 * 1. INCLUDES
 * ========================================= */
#include "tool.h"
#include "lvgl.h"
#include <math.h>                   /* Para la FPU (sqrtf) */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "qmi8658.h"             /* Descomentar para el sensor real */

/* =========================================
 * 2. MACROS Y ESTRUCTURAS DE DATOS
 * ========================================= */
#define LPF_ALPHA   0.2f            /* Suavizado del filtro EWMA */
#define THRESH_HIGH 1.25f           /* Umbral concéntrico (levantamiento) */
#define THRESH_LOW  0.85f           /* Umbral excéntrico (bajada) */

typedef enum { REP_IDLE, REP_LIFTING, REP_LOWERING } rep_state_t;

typedef struct {
    int reps;
    float magnitude;
} imu_data_t;

/* =========================================
 * 3. VARIABLES ESTÁTICAS GLOBALES
 * ========================================= */
/* Interfaz Gráfica */
static lv_obj_t *arc_force = NULL;
static lv_obj_t *label_reps = NULL;
static lv_obj_t *label_desc = NULL;
static lv_timer_t *ui_timer = NULL;

/* Tarea e IPC (Inter-Process Communication) */
static QueueHandle_t imu_queue = NULL;
static TaskHandle_t imu_task_handle = NULL;
static volatile bool task_running = false;

/* Lógica de Negocio */
static rep_state_t current_state = REP_IDLE;
static int rep_count = 0;

/* =========================================
 * 4. EL PRODUCTOR: Tarea de FreeRTOS
 * ========================================= */
static void imu_processing_task(void *pvParameters) {
    imu_data_t current_data;
    static float flt_x = 0.0f, flt_y = 0.0f, flt_z = 0.0f;
    bool first_read = true;

    while (task_running) {
        /* A. Leer Sensor (Reemplazar con lectura real) */
        float raw_x = 0.0f, raw_y = 0.0f, raw_z = 0.0f;

        qmi8658_read_accel(&raw_x, &raw_y, &raw_z);
        /* B. Filtro Paso Bajo (EWMA) */
        if (first_read) {
            flt_x = raw_x; flt_y = raw_y; flt_z = raw_z;
            first_read = false;
        } else {
            flt_x = (LPF_ALPHA * raw_x) + ((1.0f - LPF_ALPHA) * flt_x);
            flt_y = (LPF_ALPHA * raw_y) + ((1.0f - LPF_ALPHA) * flt_y);
            flt_z = (LPF_ALPHA * raw_z) + ((1.0f - LPF_ALPHA) * flt_z);
        }

        /* C. Magnitud Vectorial (FPU) */
        float mag = sqrtf((flt_x * flt_x) + (flt_y * flt_y) + (flt_z * flt_z));

        /* D. Máquina de Estados (Conteo) */
        switch (current_state) {
            case REP_IDLE:     if (mag > THRESH_HIGH) current_state = REP_LIFTING; break;
            case REP_LIFTING:  if (mag < THRESH_LOW)  current_state = REP_LOWERING; break;
            case REP_LOWERING: 
                if (mag > 0.95f && mag < 1.05f) {
                    rep_count++;
                    current_state = REP_IDLE;
                }
                break;
        }

        /* E. Enviar a Interfaz */
        current_data.reps = rep_count;
        current_data.magnitude = mag;
        xQueueOverwrite(imu_queue, &current_data);

        vTaskDelay(pdMS_TO_TICKS(20)); /* 50 Hz */
    }
    vTaskDelete(NULL);
}

/* =========================================
 * 5. EL CONSUMIDOR: Actualización LVGL
 * ========================================= */
static void ui_update_timer_cb(lv_timer_t *timer) {
    imu_data_t data;
    if (xQueueReceive(imu_queue, &data, 0) == pdTRUE) {
        /* Actualizar Texto */
        lv_label_set_text_fmt(label_reps, "%d", data.reps);

        /* Actualizar Arco (Mapeo de 1G a 2G -> 0% a 100%) */
        float extra_force = data.magnitude - 1.0f;
        if (extra_force < 0.0f) extra_force = 0.0f;
        
        int arc_pct = (int)(extra_force * 100.0f);
        if (arc_pct > 100) arc_pct = 100;
        lv_arc_set_value(arc_force, arc_pct);
    }
}

/* =========================================
 * 6. CICLO DE VIDA (Tool Interface)
 * ========================================= */
static void gym_tool_open(lv_obj_t *parent) {
    /* Reiniciar variables de lógica */
    rep_count = 0;
    current_state = REP_IDLE;

    /* Crear UI: Arco */
    arc_force = lv_arc_create(parent);
    lv_obj_set_size(arc_force, 220, 220);
    lv_arc_set_rotation(arc_force, 270);
    lv_arc_set_bg_angles(arc_force, 0, 360);
    lv_arc_set_range(arc_force, 0, 100);
    lv_obj_remove_style(arc_force, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc_force, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(arc_force);

    /* Crear UI: Textos */
    label_reps = lv_label_create(parent);
    lv_label_set_text(label_reps, "0");
    lv_obj_set_style_text_font(label_reps, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(label_reps, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align_to(label_reps, arc_force, LV_ALIGN_CENTER, 0, 0);
    lv_obj_center(label_reps);

    label_desc = lv_label_create(parent);
    lv_label_set_text(label_desc, "REPS");
    lv_obj_set_style_text_color(label_desc, lv_color_hex(0x888888), 0);
    lv_obj_align_to(label_desc, label_reps, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

    /* Levantar FreeRTOS */
    imu_queue = xQueueCreate(1, sizeof(imu_data_t));
    task_running = true;
    xTaskCreatePinnedToCore(imu_processing_task, "imu_task", 4096, NULL, 5, &imu_task_handle, 1);

    /* Levantar LVGL Timer */
    ui_timer = lv_timer_create(ui_update_timer_cb, 33, NULL);
}

static void gym_tool_close(void) {
    if (ui_timer) { lv_timer_del(ui_timer); ui_timer = NULL; }
    
    if (imu_task_handle) {
        task_running = false;
        vTaskDelay(pdMS_TO_TICKS(50));
        imu_task_handle = NULL;
    }
    
    if (imu_queue) { vQueueDelete(imu_queue); imu_queue = NULL; }
}

/* =========================================
 * 7. EXPORTACIÓN DE LA HERRAMIENTA
 * ========================================= */
const tool_t tool_gym = {
    .name = "Gym Tracker",
    .icon = LV_SYMBOL_REFRESH, 
    .accent = 0x3498DB,
    .open = gym_tool_open,
    .close = gym_tool_close
};