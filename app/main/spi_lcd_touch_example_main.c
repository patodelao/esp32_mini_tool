/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "esp_sntp.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"

#include <time.h>
#include <sys/time.h>


#include "esp_lcd_gc9a01.h"
#include "esp_lcd_touch_cst816s.h"


static const char *TAG = "example";

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           10
#define EXAMPLE_PIN_NUM_MOSI           11
#define EXAMPLE_PIN_NUM_MISO           12

#define EXAMPLE_PIN_NUM_LCD_DC         8
#define EXAMPLE_PIN_NUM_LCD_RST        14
#define EXAMPLE_PIN_NUM_LCD_CS         9

#define EXAMPLE_PIN_NUM_BK_LIGHT       2
#define EXAMPLE_PIN_NUM_TOUCH_CS       -1

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              320
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_GC9A01
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              240
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

#define CONFIG_LCD_TOUCH_RST           13
#define CONFIG_LCD_TOUCH_INT           5
#define CONFIG_DISPLAY_CST816S_SDA     6   
#define CONFIG_DISPLAY_CST816S_SCL     7

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_SSID "invitado"
#define WIFI_PASS "111111111111111"
static bool sntp_initialized = false;

static EventGroupHandle_t wifi_event_group;
static lv_obj_t *wifi_btn;
static bool wifi_connected = false;

static SemaphoreHandle_t lvgl_mux = NULL;

static lv_obj_t *clock_label;

esp_lcd_touch_handle_t tp = NULL;
static SemaphoreHandle_t touch_mux = NULL;

void connect_to_wifi(void);

extern void create_gradient_circle();
extern void create_main_ui();

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    }
}


static void example_lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/*
    Define a callback function used by ISR.
*/
static void touch_callback(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

#define I2C_MASTER_NUM I2C_NUM_0 // Define the I2C port number

void i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_DISPLAY_CST816S_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num =  CONFIG_DISPLAY_CST816S_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
}



static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


esp_err_t cst816s_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,      // Puerto I2C
        0x15,               // Dirección del dispositivo CST816S
        &reg_addr,             // Dirección del registro a leer
        1,                     // Longitud del registro (1 byte)
        data,                  // Buffer donde guardar los datos leídos
        len,                   // Cuántos bytes leer
        pdMS_TO_TICKS(100)     // Timeout
    );
}


// Función para inicializar el táctil CST816S
esp_err_t cst816s_init() {
    // Realizar una lectura simple para verificar la comunicación
    uint8_t reg = 0x00;  // Registro de ejemplo para verificar la respuesta
    uint8_t data;
    esp_err_t ret = cst816s_read(reg, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer del CST816S: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "CST816S respondió correctamente en la lectura inicial");

    // Configuración adicional según el dispositivo (si es necesario)
    // Por ejemplo, activar el modo táctil o configurar interrupciones

    return ESP_OK;
}



void uart_init(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void send_ok_event_cb(lv_event_t *e)
{
    const char *msg = "OK\n";
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
    connect_to_wifi();
}





////////////////////////////////////////////////////////////////////////////////////
static void update_wifi_button_style()
{
    lv_obj_set_style_bg_color(wifi_btn, wifi_connected ? lv_color_hex(0x00FF00) : lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_radius(wifi_btn, LV_RADIUS_CIRCLE, 0);
    lv_label_set_text(lv_obj_get_child(wifi_btn, 0), wifi_connected ? "Conectado" : "Desconectado");
}


static void wifi_toggle_btn_cb(lv_event_t *e)
{
    if (!example_lvgl_lock(1000)) return;

    if (!wifi_connected) {
        ESP_LOGI("WIFI", "Intentando conectar...");
        connect_to_wifi();  // Solo llama a esp_wifi_connect()

        // Espera conexión con timeout
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));

        if (bits & WIFI_CONNECTED_BIT) {
            wifi_connected = true;
            uart_write_bytes(UART_NUM_0, "OK\n", 3);
            ESP_LOGI("WIFI", "Conectado exitosamente.");
        } else {
            ESP_LOGE("WIFI", "No se pudo conectar a Wi-Fi.");
        }

    } else {
        ESP_LOGI("WIFI", "Desconectando Wi-Fi...");
        esp_wifi_disconnect();
        wifi_connected = false;
        ESP_LOGI("WIFI", "Wi-Fi desconectado.");
    }

    update_wifi_button_style();  // Refresca color y texto del botón
    example_lvgl_unlock();
}


////////////////////////////////////////////////////////////////////////////////////


void create_main_ui(void) {
    // Crear contenedor base para toda la interfaz
    lv_obj_t * main_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(main_container, 240, 240);
    lv_obj_center(main_container);
    lv_obj_set_scrollbar_mode(main_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(main_container, lv_color_hex(0x00CFFF), 0);
    lv_obj_set_style_bg_grad_color(main_container, lv_color_hex(0x004E92), 0);
    lv_obj_set_style_bg_grad_dir(main_container, LV_GRAD_DIR_VER, 0);
    lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

    // Crear header (por ejemplo, 30px de alto)
    lv_obj_t * header = lv_obj_create(main_container);
    lv_obj_set_size(header, 240, 30);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x003B5C), 0);
    lv_obj_set_style_text_color(header, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    // Crear la etiqueta del reloj (persistente)
    clock_label = lv_label_create(header);
    lv_label_set_text(clock_label, "");
    lv_obj_align(clock_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(clock_label, &lv_font_montserrat_14, 0);

    // Contenedor para las pantallas desplazables debajo del header
    lv_obj_t * screen_area = lv_obj_create(main_container);
    lv_obj_set_size(screen_area, 240, 210); // 240 - 30 (header)
    lv_obj_align(screen_area, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(screen_area, LV_OPA_TRANSP, 0);
    lv_obj_set_scroll_snap_x(screen_area, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_snap_y(screen_area, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_dir(screen_area, LV_DIR_ALL);
    lv_obj_set_scrollbar_mode(screen_area, LV_SCROLLBAR_MODE_OFF);

    // Crear 4 pantallas de ejemplo (arriba, abajo, izquierda, derecha)
    lv_obj_t * screen1 = lv_obj_create(screen_area);
    lv_obj_set_size(screen1, 240, 210);
    lv_obj_align(screen1, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(screen1, lv_palette_lighten(LV_PALETTE_GREEN, 2), 0);

    lv_obj_t * screen2 = lv_obj_create(screen_area);
    lv_obj_set_size(screen2, 240, 210);
    lv_obj_align(screen2, LV_ALIGN_OUT_RIGHT_MID, 240, 0); // derecha
    lv_obj_set_style_bg_color(screen2, lv_palette_lighten(LV_PALETTE_RED, 2), 0);

    lv_obj_t * screen3 = lv_obj_create(screen_area);
    lv_obj_set_size(screen3, 240, 210);
    lv_obj_align(screen3, LV_ALIGN_OUT_LEFT_MID, -240, 0); // izquierda
    lv_obj_set_style_bg_color(screen3, lv_palette_lighten(LV_PALETTE_BLUE, 2), 0);

    lv_obj_t * screen4 = lv_obj_create(screen_area);
    lv_obj_set_size(screen4, 240, 210);
    lv_obj_align(screen4, LV_ALIGN_OUT_BOTTOM_MID, 0, 210); // abajo
    lv_obj_set_style_bg_color(screen4, lv_palette_lighten(LV_PALETTE_YELLOW, 2), 0);

    // Puedes agregar botones u otros elementos en estas pantallas
    wifi_btn = lv_btn_create(screen1);
    lv_obj_center(wifi_btn);
    lv_obj_add_event_cb(wifi_btn, wifi_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_size(wifi_btn, 120, 50);

    lv_obj_t * label = lv_label_create(wifi_btn);
    lv_label_set_text(label, "Desconectado");
    lv_obj_center(label);

    update_wifi_button_style();
}






///////////// wifi /////////////

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
esp_wifi_connect();
} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
ESP_LOGI("WIFI", "Desconectado, reintentando...");
esp_wifi_connect();
} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
ESP_LOGI("WIFI", "Dirección IP: " IPSTR, IP2STR(&event->ip_info.ip));
xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
}
}

/*
void connect_to_wifi(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Esperar a que se conecte
    ESP_LOGI("WIFI", "Conectando a %s...", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WIFI", "Conectado a %s", WIFI_SSID);
        printf("Conectado a %s\n", WIFI_SSID);
    } else {
        ESP_LOGE("WIFI", "Error al conectar a %s", WIFI_SSID);
        printf("Error al conectar a %s\n", WIFI_SSID);
    }
}
*/

void wifi_status_task(void *pvParameters)
{
    while (1) {
        const char *msg = wifi_connected ? "Wi-Fi: Conectado\n" : "Wi-Fi: Desconectado\n";
        uart_write_bytes(UART_NUM_0, msg, strlen(msg));
        vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1 segundo
    }
}


void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());  // <- esto SOLO UNA VEZ

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Declare and initialize wifi_config
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Crea el grupo de eventos
    wifi_event_group = xEventGroupCreate();

    // Registra handlers de eventos (una sola vez también)
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
}
void initialize_sntp()
{
    if (sntp_initialized) return;

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    sntp_initialized = true;
}

void sntp_sync_task(void *arg)
{
    time_t now;
    struct tm timeinfo;
    int retry = 0;
    const int retry_count = 10;

    do {
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year >= (2020 - 1900)) {
            ESP_LOGI("SNTP", "Hora sincronizada: %s", asctime(&timeinfo));
            break;
        }
        ESP_LOGI("SNTP", "Esperando sincronización SNTP... (%d/%d)", retry + 1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
    } while (++retry < retry_count);

    if (retry == retry_count) {
        ESP_LOGW("SNTP", "No se logró sincronizar la hora");
    }

    vTaskDelete(NULL);
}


void connect_to_wifi(void)
{
    ESP_LOGI("WIFI", "Conectando a red Wi-Fi...");
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Configura zona horaria de Santiago de Chile


    // Iniciar SNTP
    initialize_sntp();
    setenv("TZ", "CLT4CLST,M9.1.6/23,M4.1.6/23", 1);
    //setenv("TZ", "CLT4CLST,m10.1.0/0,m4.1.0/0", 1);
    tzset();
    // Crear tarea para esperar sincronización sin bloquear
    xTaskCreate(sntp_sync_task, "sntp_sync_task", 4096, NULL, 3, NULL);
}



void disconnect_wifi(void)
{
    ESP_LOGI("WIFI", "Desconectando...");
    esp_wifi_disconnect();
}









//////////////////// reloj /////////////////

void create_clock_label()
{
    clock_label = lv_label_create(lv_scr_act());
    lv_label_set_text(clock_label, ""); // Mostrar vacío inicialmente
    lv_obj_align(clock_label, LV_ALIGN_TOP_MID, 0, 5); // Arriba al centro, con 10px de margen
    lv_obj_set_style_text_font(clock_label, &lv_font_montserrat_14, 0); // Fuente legible
}


void clock_task(void *pvParameters)
{
    char time_str[64];
    time_t now;
    struct tm timeinfo;

    while (1) {
        time(&now);
        localtime_r(&now, &timeinfo);

        strftime(time_str, sizeof(time_str), "%H:%M:%S", &timeinfo);
        if (clock_label) {
            lv_label_set_text(clock_label, time_str);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



void wait_for_time_sync()
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI("SNTP", "Esperando sincronización de hora...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2020 - 1900)) {
        ESP_LOGI("SNTP", "Hora sincronizada correctamente");
    } else {
        ESP_LOGE("SNTP", "Fallo en sincronización NTP");
    }
}








void app_main(void){
    // ------------------------- LVGL: Buffers y Driver -------------------------
    static lv_disp_draw_buf_t disp_buf; // contiene los buffers gráficos internos
    static lv_disp_drv_t disp_drv;      // contiene las funciones callback del driver LVGL

    // ------------------------- Sincronización para Touch -------------------------
    // Crear un mutex binario para proteger el acceso al táctil
    touch_mux = xSemaphoreCreateBinary();
    assert(touch_mux);  

    // ------------------------- Inicializa NVS -------------------------
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);



    // ------------------------- Inicialización I2C -------------------------
    i2c_init();
    // ---------------------------- Inicialización UART ---------------------
    uart_init();
    wifi_init(); // Inicializa Wi-Fi
    // ------------------------- Inicialización del panel táctil CST816S -------------------------
    esp_err_t err = cst816s_init();
    if (err != ESP_OK) {
        ESP_LOGE("CST816S", "Error al inicializar CST816S: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("CST816S", "Táctil CST816S inicializado correctamente");
    }

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 240,
        .y_max = 240,
        .rst_gpio_num = 13,
        .int_gpio_num = 5,
        .levels = {.reset = 0, .interrupt = 0},
        .flags = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0},
        .interrupt_callback = touch_callback,
    };

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)0 , &tp_io_config, &tp_io_handle);
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);     

    // ------------------------- Control de Backlight del LCD -------------------------
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    // ------------------------- Inicialización del bus SPI para el LCD -------------------------
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // ------------------------- Configuración del panel LCD GC9A01 -------------------------
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // ------------------------- Encender retroiluminación -------------------------
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    // ------------------------- Inicialización de LVGL -------------------------
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // Buffers de dibujo DMA para LVGL
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    // Registro del driver de display en LVGL
    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    // ------------------------- Timer periódico para LVGL -------------------------
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    // ------------------------- Registro de entrada táctil en LVGL -------------------------
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);



    // ------------------------- TAREAS -------------------------


    // ------------------------- Tarea de renderizado para LVGL -------------------------
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    // ------------------------- Tarea de reloj -------------------------
    //create_clock_label();
    xTaskCreate(clock_task, "clock_task", 4096, NULL, 4, NULL);


    // ------------------------- Tarea de estado de wifi -------------------------
    xTaskCreate(wifi_status_task, "wifi_status_task", 2048, NULL, 5, NULL);

    // ------------------------- Mostrar interfaz LVGL inicial -------------------------
    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    if (example_lvgl_lock(-1)) {
        create_main_ui(); // o create_gradient_circle();
        create_clock_label();
        example_lvgl_unlock();
    }
}
