/*
 * main.c — Punto de entrada del ESP32 mini tool.
 * Inicializa hardware (bsp) y Wi-Fi, y muestra el menú de herramientas.
 */
#include "esp_log.h"
#include <time.h>
#include <sys/time.h>

#include "esp_timer.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "bsp.h"
#include "wifi_manager.h"
#include "qmi8658.h"
#include "ui_menu.h"
#include "ui_notify.h"
#include "ui_power.h"
#include "mqtt_hub.h"
#include "alert_service.h"
#include "fleet_service.h"
#include "sensor_service.h"
#include "self_node.h"
#include "web_ui.h"
#include "weather_service.h"
#include "pedometer_service.h"
#include "alarm_clock.h"

static const char *TAG = "app";

/* ==========================================================
 * RUTINA DE AUTOGUARDADO DE TIEMPO EN NVS
 * ========================================================== */

/* 1. Callback que se dispara automáticamente cuando SNTP consigue la hora de internet */
static void time_sync_notification_cb(struct timeval *tv) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i64(my_handle, "epoch_time", (int64_t)tv->tv_sec);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Epoch actualizado desde internet y guardado en NVS: %lld", (long long)tv->tv_sec);
    }
}

/* NUEVO: Guardado periódico cada 60 segundos */
static void periodic_time_save_cb(void *arg) {
    time_t now;
    time(&now);
    
    /* Solo guardamos si ya tenemos una hora válida (> año 2020) */
    if (now > 1600000000) {
        nvs_handle_t my_handle;
        if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
            nvs_set_i64(my_handle, "epoch_time", (int64_t)now);
            nvs_commit(my_handle);
            nvs_close(my_handle);
        }
    }
}

/* 2. Rutina de inicialización de tiempo (llamarla en app_main) */
static void time_keeper_init(void) {
    /* Enganchar el guardado automático al motor SNTP */
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    /* Al encender, leemos la memoria estática (NVS) para ver si hay un Epoch guardado */
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        int64_t saved_epoch = 0;
        if (nvs_get_i64(my_handle, "epoch_time", &saved_epoch) == ESP_OK) {
            if (saved_epoch > 1600000000) {
                /* Inyectamos el Epoch guardado al reloj por hardware del ESP32 */
                struct timeval tv = { .tv_sec = (time_t)saved_epoch, .tv_usec = 0 };
                settimeofday(&tv, NULL);
                ESP_LOGI(TAG, "Epoch rescatado de memoria estática: %lld", (long long)saved_epoch);
            }
        }
        nvs_close(my_handle);
    }

    /* Levantar timer periódico de 1 minuto (60.000.000 microsegundos) */
    const esp_timer_create_args_t timer_args = {
        .callback = &periodic_time_save_cb,
        .name = "nvs_time_saver"
    };
    esp_timer_handle_t save_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &save_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(save_timer, 60000000ULL));
}


/* ==========================================================
 * APP MAIN
 * ========================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando ESP32 mini tool");
    
    /* NVS + red (sin conectar aún) */
    wifi_manager_init(); 
    
    /* Configurar zona horaria de Chile Continental (CLT/CLST) */
    setenv("TZ", "CLT4CLST,M9.1.0/0,M4.1.0/0", 1);
    tzset();

    /* Iniciar rutinas globales de tiempo (Autoguardado) */
    time_keeper_init();

    /* SPI/LCD + I2C/touch + LVGL */
    bsp_init(); 

    /* IMU (comparte el I2C del touch). Si falta, la tool Nivel lo indica. */
    if (qmi8658_init() != ESP_OK) {
        ESP_LOGW(TAG, "IMU no disponible");
    }

    /* Construir la UI inicial bajo el lock de LVGL */
    if (bsp_lvgl_lock(-1)) {
        /* Fondo base oscuro (navy del watchface) para todas las tools y el menú.
           Se fija una vez; lv_obj_clean no borra el estilo propio de la pantalla. */
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000814), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);

        ui_notify_init();          /* notificaciones flotantes */
        ui_power_init();           /* apagado de pantalla + despertar por gesto */
        pedometer_service_init();  /* contador de pasos siempre activo */
        alarm_clock_init();        /* despertador: suena con cualquier tool abierta */
        create_main_menu();
        bsp_lvgl_unlock();
    }

    /* Cliente MQTT compartido (alertas, fleet, sensores, control). */
    mqtt_hub_init();

    /* Servicio de alertas en segundo plano (refri y futuros equipos).
       Se conecta cuando haya Wi-Fi; enruta las alertas a ui_notify. */
    alert_service_init();

    /* Registro de nodos del home-lab (tool Fleet). */
    fleet_service_init();

    /* Lecturas de sensores del home-lab (tool Sensores). */
    sensor_service_init();

    /* El propio minitool se publica como un nodo mas del home-lab. */
    self_node_init();

    /* Panel web del home-lab y actualizacion por WiFi del propio reloj. */
    web_ui_start();

    /* Servicio de clima con caché compartida (tool Clima + screensaver). */
    weather_service_init();
}