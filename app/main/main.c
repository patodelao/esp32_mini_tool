/*
 * main.c — Punto de entrada del ESP32 mini tool.
 * Inicializa hardware (bsp) y Wi-Fi, y muestra el menú de herramientas.
 */
#include "esp_log.h"

#include "bsp.h"
#include "wifi_manager.h"
#include "qmi8658.h"
#include "ui_menu.h"

static const char *TAG = "app";

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando ESP32 mini tool");

    wifi_manager_init();  /* NVS + red (sin conectar aún) */
    bsp_init();           /* SPI/LCD + I2C/touch + LVGL */

    /* IMU (comparte el I2C del touch). Si falta, la tool Nivel lo indica. */
    if (qmi8658_init() != ESP_OK) {
        ESP_LOGW(TAG, "IMU no disponible");
    }

    /* Construir la UI inicial bajo el lock de LVGL */
    if (bsp_lvgl_lock(-1)) {
        ui_menu_show();
        bsp_lvgl_unlock();
    }
}
