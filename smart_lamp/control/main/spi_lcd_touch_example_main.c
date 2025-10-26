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
#include "lvgl.h" // <-- Make sure you have this dependency
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "esp_sntp.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include <time.h>
#include <sys/time.h>

// --- Inclusión para el Cliente HTTP ---
#include "esp_http_client.h"

#include "esp_lcd_gc9a01.h"
#include "esp_lcd_touch_cst816s.h"


static const char *TAG = "EMISOR_CON_SWITCH"; // Tag updated

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Configuración del LCD (No changes) /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ      (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL   1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK            10
#define EXAMPLE_PIN_NUM_MOSI            11
#define EXAMPLE_PIN_NUM_MISO            12
#define EXAMPLE_PIN_NUM_LCD_DC          8
#define EXAMPLE_PIN_NUM_LCD_RST         14
#define EXAMPLE_PIN_NUM_LCD_CS          9
#define EXAMPLE_PIN_NUM_BK_LIGHT        2
#define EXAMPLE_PIN_NUM_TOUCH_CS        -1
#define EXAMPLE_LCD_H_RES               240
#define EXAMPLE_LCD_V_RES               240
#define EXAMPLE_LCD_CMD_BITS            8
#define EXAMPLE_LCD_PARAM_BITS          8
#define EXAMPLE_LVGL_TICK_PERIOD_MS     2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2
#define CONFIG_LCD_TOUCH_RST            13
#define CONFIG_LCD_TOUCH_INT            5
#define CONFIG_DISPLAY_CST816S_SDA      6
#define CONFIG_DISPLAY_CST816S_SCL      7


// --- ¡IMPORTANTE! EDITA ESTAS LÍNEAS ---
// Credenciales Wi-Fi (deben coincidir con el Receptor)
#define WIFI_SSID "WiFi_Mesh-190685"
#define WIFI_PASS "c9KsQYCy"
// IP del Receptor (mira el log del Receptor para obtenerla)
#define RECEPTOR_IP "192.168.1.31" // <--- ¡¡¡CAMBIA ESTO!!!
// --- Fin de sección importante ---


#define WIFI_CONNECTED_BIT BIT0
static bool sntp_initialized = false;

static EventGroupHandle_t wifi_event_group;
static volatile bool wifi_connected = false; // Flag for Wi-Fi status

static SemaphoreHandle_t lvgl_mux = NULL;

esp_lcd_touch_handle_t tp = NULL;
static SemaphoreHandle_t touch_mux = NULL;

// Forward declarations for HTTP trigger functions
void trigger_remote_next_state(void);
void trigger_remote_light_on(void);
void trigger_remote_light_off(void);

// --- Hardware and LVGL initialization functions (No changes) ---
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
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false); esp_lcd_panel_mirror(panel_handle, true, false);
        esp_lcd_touch_set_mirror_y(tp, false); esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, true); esp_lcd_panel_mirror(panel_handle, true, true);
        esp_lcd_touch_set_mirror_y(tp, false); esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false); esp_lcd_panel_mirror(panel_handle, false, true);
        esp_lcd_touch_set_mirror_y(tp, false); esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, true); esp_lcd_panel_mirror(panel_handle, false, false);
        esp_lcd_touch_set_mirror_y(tp, false); esp_lcd_touch_set_mirror_x(tp, false);
        break;
    }
}
static void example_lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0}; uint16_t touchpad_y[1] = {0}; uint8_t touchpad_cnt = 0;
    esp_lcd_touch_read_data(drv->user_data);
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0]; data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
static void touch_callback(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
#define I2C_MASTER_NUM I2C_NUM_0
void i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_DISPLAY_CST816S_SDA, .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num =  CONFIG_DISPLAY_CST816S_SCL, .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
}
static void example_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}
bool example_lvgl_lock(int timeout_ms)
{
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
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
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
    return i2c_master_write_read_device( I2C_MASTER_NUM, 0x15, &reg_addr, 1, data, len, pdMS_TO_TICKS(100) );
}
esp_err_t cst816s_init() {
    uint8_t reg = 0x00; uint8_t data;
    esp_err_t ret = cst816s_read(reg, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading from CST816S: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "CST816S responded correctly during initial read");
    return ESP_OK;
}
// ----------------------------------------------------------------


// --- HTTP Client Section ---
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_ON_DATA:
            // Log response data from the receiver
            ESP_LOGI(TAG, "Receptor Response: %.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        default:
            // Handle other events like HEADERS_SENT, ON_HEADER, etc. if needed
            break;
    }
    return ESP_OK;
}

// Helper function to send GET requests
static void send_http_get_request(const char *path)
{
    if (!wifi_connected) {
        ESP_LOGW(TAG, "Cannot send HTTP request to '%s', Wi-Fi disconnected.", path);
        return;
    }

    char receptor_url[100]; // Slightly larger buffer
    snprintf(receptor_url, sizeof(receptor_url), "http://%s%s", RECEPTOR_IP, path);

    ESP_LOGI(TAG, "Sending request to: %s", receptor_url);

    esp_http_client_config_t config = {
        .url = receptor_url,
        .event_handler = _http_event_handler,
        .timeout_ms = 5000, // 5 second timeout
        .method = HTTP_METHOD_GET, // Explicitly set GET
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP GET request to '%s' successful, status = %d", path, esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request to '%s' failed: %s", path, esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

// Functions to trigger specific remote actions
void trigger_remote_next_state(void) {
    send_http_get_request("/next-state");
}
void trigger_remote_light_on(void) {
    send_http_get_request("/light/on");
}
void trigger_remote_light_off(void) {
    send_http_get_request("/light/off");
}
// --- End HTTP Client Section ---


// --- LVGL Callbacks ---
// Callback for the Red "NEXT" button
static void remote_trigger_btn_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Remote trigger button pressed.");
    trigger_remote_next_state(); // Call the refactored function
}

// NEW! Callback for the White Light Switch
static void white_light_switch_cb(lv_event_t *e)
{
    lv_obj_t * sw = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        ESP_LOGI(TAG, "White Light Switch changed state.");
        if (lv_obj_has_state(sw, LV_STATE_CHECKED)) { // If switch is NOW checked (ON)
            ESP_LOGI(TAG, "Calling trigger_remote_light_on()");
            trigger_remote_light_on();
        } else { // If switch is NOW unchecked (OFF)
            ESP_LOGI(TAG, "Calling trigger_remote_light_off()");
            trigger_remote_light_off();
        }
    }
}
// --- End LVGL Callbacks ---


// --- Interface Creation (Modified) ---
void create_simple_button_ui(void)
{
    // Dark background
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x111111), LV_STATE_DEFAULT);

    // --- Red Button ---
    lv_obj_t * remote_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(remote_btn, 180, 180);
    // Move slightly up to make space for the switch
    lv_obj_align(remote_btn, LV_ALIGN_CENTER, 0, -30); // Position adjustment
    lv_obj_add_event_cb(remote_btn, remote_trigger_btn_cb, LV_EVENT_CLICKED, NULL);

    // Button Style (Red, round, raised effect)
    lv_obj_set_style_radius(remote_btn, LV_RADIUS_CIRCLE, LV_STATE_DEFAULT);
    // Normal State
    lv_obj_set_style_bg_color(remote_btn, lv_palette_main(LV_PALETTE_RED), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(remote_btn, lv_palette_darken(LV_PALETTE_RED, 2), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(remote_btn, LV_GRAD_DIR_VER, LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(remote_btn, lv_palette_lighten(LV_PALETTE_RED, 1), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(remote_btn, 3, LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(remote_btn, 5, LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(remote_btn, lv_palette_darken(LV_PALETTE_RED, 4), LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(remote_btn, 3, LV_STATE_DEFAULT);
    // Pressed State (Darker, sunken effect)
    lv_obj_set_style_bg_color(remote_btn, lv_palette_darken(LV_PALETTE_RED, 2), LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_color(remote_btn, lv_palette_darken(LV_PALETTE_RED, 4), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(remote_btn, 0, LV_STATE_PRESSED);
    lv_obj_set_style_translate_y(remote_btn, 1, LV_STATE_PRESSED);

    // Button Label
    lv_obj_t * remote_label = lv_label_create(remote_btn);
    lv_label_set_text(remote_label, "NEXT"); // Changed for clarity
    lv_obj_set_style_text_font(remote_label, &lv_font_montserrat_14, LV_STATE_DEFAULT);
    lv_obj_center(remote_label);

    // --- NEW! White Light Switch ---
    lv_obj_t * light_sw = lv_switch_create(lv_scr_act());
    // Position below the red button
    lv_obj_align(light_sw, LV_ALIGN_CENTER, 0, 80); // Position below
    // Assign the new callback
    lv_obj_add_event_cb(light_sw, white_light_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // --- NEW! Label for the Switch ---
    lv_obj_t * light_label = lv_label_create(lv_scr_act());
    lv_label_set_text(light_label, "Luz Blanca:");
    // Align label to the left of the switch
    lv_obj_align_to(light_label, light_sw, LV_ALIGN_OUT_LEFT_MID, -10, 0); // To the left
}
// --- End Interface Creation ---


// --- SNTP Functions (No changes) ---
void initialize_sntp()
{
    if (sntp_initialized) return;
    ESP_LOGI(TAG, "Initializing SNTP...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    sntp_initialized = true;
}

void sntp_sync_task(void *arg)
{
    time_t now; struct tm timeinfo;
    int retry = 0; const int retry_count = 10;
    do {
        time(&now);
        localtime_r(&now, &timeinfo);
        if (timeinfo.tm_year >= (2020 - 1900)) {
            ESP_LOGI("SNTP", "Time synchronized: %s", asctime(&timeinfo));
            break;
        }
        ESP_LOGI("SNTP", "Waiting for SNTP synchronization... (%d/%d)", retry + 1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (++retry < retry_count);
    if (retry == retry_count) {
        ESP_LOGW("SNTP", "Failed to synchronize time");
    }
    vTaskDelete(NULL);
}
// --- End SNTP Functions ---


// --- Wi-Fi Initialization and Event Handler (No changes) ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI("WIFI", "Disconnected, retrying...");
        wifi_connected = false; // Mark as disconnected
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

        wifi_connected = true; // Mark as connected

        // Initialize SNTP once IP is obtained
        initialize_sntp();
        setenv("TZ", "CLT4CLST,M9.1.6/23,M4.1.6/23", 1); // Chile Timezone
        tzset();
        xTaskCreate(sntp_sync_task, "sntp_sync_task", 4096, NULL, 3, NULL);
    }
}

void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    wifi_event_group = xEventGroupCreate(); // Create event group before registering handlers

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_start()); // This starts the connection attempt automatically
    ESP_LOGI(TAG, "wifi_init configured for automatic connection.");
}
// --- End Wi-Fi Section ---


// --- app_main (No major changes, ensures UI function is called) ---
void app_main(void){
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

    touch_mux = xSemaphoreCreateBinary();
    assert(touch_mux);

    // --- 1. Initialize NVS ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --- 2. Initialize Peripherals ---
    i2c_init(); // For touch controller
    wifi_init(); // Starts Wi-Fi connection attempts

    // --- 3. Initialize Touch ---
    esp_err_t err = cst816s_init();
    if (err != ESP_OK) {
        ESP_LOGE("CST816S", "Error initializing CST816S: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("CST816S", "Touch CST816S initialized correctly");
    }
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = 240, .y_max = 240,
        .rst_gpio_num = 13, .int_gpio_num = 5,
        .levels = {.reset = 0, .interrupt = 0},
        .flags = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0},
        .interrupt_callback = touch_callback,
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)0 , &tp_io_config, &tp_io_handle);
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);

    // --- 4. Initialize Display (Backlight, SPI, Driver) ---
    // (No changes in this section)
    gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK, .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO, .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC, .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ, .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS, .spi_mode = 0,
        .trans_queue_depth = 10, .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST, .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    // --- 5. Initialize LVGL ---
    // (No changes in this section)
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);
    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES; disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf; disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick, .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);

    // --- 6. Create Tasks ---
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Creating LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    // --- 7. Create User Interface ---
    ESP_LOGI(TAG, "Displaying Simple Button UI");
    if (example_lvgl_lock(-1)) {
        create_simple_button_ui(); // <-- Call the function that creates button AND switch
        example_lvgl_unlock();
    }
}