/*
 * bsp.c — Implementación del Board Support Package.
 * La secuencia de init se basa en el ejemplo original spi_lcd_touch, adaptada
 * a la placa Waveshare ESP32-S3-Touch-LCD-1.28 (GC9A01 + CST816S).
 */
#include "bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_touch_cst816s.h"

static const char *TAG = "bsp";

/* ------------------------- Pines (Waveshare 1.28") ------------------------- */
#define PIN_SCLK      10
#define PIN_MOSI      11
#define PIN_MISO      12
#define PIN_LCD_DC    8
#define PIN_LCD_RST   14
#define PIN_LCD_CS    9
#define PIN_BK_LIGHT  2   /* backlight, manejado por PWM (LEDC) */
#define PIN_TOUCH_RST 13
#define PIN_TOUCH_INT 5

#define LCD_HOST              SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ    (20 * 1000 * 1000)
#define LCD_CMD_BITS          8
#define LCD_PARAM_BITS        8
#define LCD_BK_LIGHT_ON_LEVEL 1

/* ------------------------- Parámetros de LVGL ------------------------- */
#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2
#define LVGL_DRAW_BUF_LINES    20

static esp_lcd_touch_handle_t s_tp = NULL;
static SemaphoreHandle_t s_lvgl_mux = NULL;
static SemaphoreHandle_t s_touch_mux = NULL;

esp_lcd_touch_handle_t bsp_touch_handle(void) { return s_tp; }

bool bsp_lvgl_lock(int timeout_ms)
{
    const TickType_t ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(s_lvgl_mux, ticks) == pdTRUE;
}

void bsp_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(s_lvgl_mux);
}

/* ------------------------- Callbacks del display ------------------------- */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io,
                                    esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
}

/* ------------------------- Callbacks del touch ------------------------- */
static void touch_isr_cb(esp_lcd_touch_handle_t tp)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_touch_mux, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    /* Hay un toque en curso: mientras dure, seguimos sondeando para capturar
     * el movimiento y el "release" final. */
    static bool active = false;

    uint16_t x[1] = {0}, y[1] = {0};
    uint8_t cnt = 0;

    /* Solo tocamos el bus I2C si la interrupción INT avisó de actividad, o si
     * un toque sigue en curso. En reposo no leemos: el CST816S entra en
     * auto-reposo (deja de responder al I2C) y antes esas lecturas a ciegas
     * inundaban el log con "I2C read failed" cada ~30 ms. */
    if (xSemaphoreTake(s_touch_mux, 0) == pdTRUE || active) {
        esp_lcd_touch_read_data(drv->user_data);
    }

    bool pressed = esp_lcd_touch_get_coordinates(drv->user_data, x, y, NULL, &cnt, 1);
    if (pressed && cnt > 0) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        active = true;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
        active = false;
    }
}

static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");
    uint32_t delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        if (bsp_lvgl_lock(-1)) {
            delay_ms = lv_timer_handler();
            bsp_lvgl_unlock();
        }
        if (delay_ms > LVGL_TASK_MAX_DELAY_MS) delay_ms = LVGL_TASK_MAX_DELAY_MS;
        else if (delay_ms < LVGL_TASK_MIN_DELAY_MS) delay_ms = LVGL_TASK_MIN_DELAY_MS;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/* ------------------------- Inicialización de hardware ------------------------- */

/*
 * I2C con el driver nuevo (driver/i2c_master.h). El legacy (driver/i2c.h) está
 * deprecado en ESP-IDF 5.3 y lo avisa en cada arranque.
 *
 * El cambio de fondo es el modelo: antes se instalaba un driver por puerto y
 * cada quien hacía transacciones contra un número de puerto; ahora se crea un
 * BUS y cada chip se registra como dispositivo. El bus serializa los accesos
 * con su propio lock, así que el touch y el IMU pueden convivir sin que haya
 * que coordinarlos desde afuera.
 */
static i2c_master_bus_handle_t s_i2c_bus = NULL;

static void i2c_init(void)
{
    const i2c_master_bus_config_t conf = {
        .i2c_port = BSP_I2C_NUM,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &s_i2c_bus));
}

i2c_master_bus_handle_t bsp_i2c_bus(void) { return s_i2c_bus; }

static void touch_init(void)
{
    esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    /* El driver nuevo exige la velocidad por dispositivo; la macro del
     * componente no la fija. */
    io_config.scl_speed_hz = BSP_I2C_HZ;
    esp_lcd_touch_config_t cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = PIN_TOUCH_RST,
        .int_gpio_num = PIN_TOUCH_INT,
        .levels = {.reset = 0, .interrupt = 0},
        .flags = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0},
        .interrupt_callback = touch_isr_cb,
    };
    esp_lcd_panel_io_handle_t io = NULL;
    /* Pasarle el handle del bus (y no un número de puerto) es lo que hace que
     * esp_lcd use la variante nueva del driver: la macro despacha por tipo. */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(s_i2c_bus, &io_config, &io));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(io, &cfg, &s_tp));
}

/* --- Backlight por PWM ----------------------------------------------------
 *
 * Antes era un GPIO on/off y quedaba encendido siempre. Con LEDC se puede
 * atenuar y, sobre todo, apagar la pantalla cuando no la estás mirando (lo
 * hace ui_power), que es de donde sale casi todo el consumo. */
#define BK_LEDC_TIMER   LEDC_TIMER_0
#define BK_LEDC_CHANNEL LEDC_CHANNEL_0
#define BK_LEDC_BITS    LEDC_TIMER_10_BIT
#define BK_LEDC_MAX     ((1 << 10) - 1)

static int s_bk_pct = 100;

void bsp_backlight_set(int pct)
{
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    s_bk_pct = pct;

    uint32_t duty = ((uint32_t)BK_LEDC_MAX * (uint32_t)pct) / 100;
#if LCD_BK_LIGHT_ON_LEVEL == 0
    duty = BK_LEDC_MAX - duty;   /* backlight activo en bajo */
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BK_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BK_LEDC_CHANNEL);
}

int bsp_backlight_get(void) { return s_bk_pct; }

static void backlight_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BK_LEDC_BITS,
        .timer_num       = BK_LEDC_TIMER,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num   = PIN_BK_LIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BK_LEDC_CHANNEL,
        .timer_sel  = BK_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

static esp_lcd_panel_handle_t lcd_init(lv_disp_drv_t *disp_drv)
{
    backlight_init();

    /* Bus SPI */
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BSP_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* Panel IO (SPI) */
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io));

    /* Panel GC9A01 */
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io, &panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    bsp_backlight_set(100);
    return panel;
}

void bsp_init(void)
{
    /* Buffers y drivers de LVGL (estáticos: viven toda la vida del programa) */
    static lv_disp_draw_buf_t draw_buf;
    static lv_disp_drv_t disp_drv;
    static lv_indev_drv_t indev_drv;

    s_touch_mux = xSemaphoreCreateBinary();
    assert(s_touch_mux);

    i2c_init();
    touch_init();

    esp_lcd_panel_handle_t panel = lcd_init(&disp_drv);

    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(BSP_LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(BSP_LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 && buf2);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, BSP_LCD_H_RES * LVGL_DRAW_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    const esp_timer_create_args_t tick_args = {.callback = &lvgl_tick_cb, .name = "lvgl_tick"};
    esp_timer_handle_t tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = lvgl_touch_read_cb;
    indev_drv.user_data = s_tp;

    /* Descartar cualquier INT espurio disparado durante el init (el reset del
     * CST816S genera flancos en el pin INT): así el primer read no llega a un
     * chip aún no listo, evitando un "I2C read failed" suelto al arranque. */
    xSemaphoreTake(s_touch_mux, 0);

    lv_indev_drv_register(&indev_drv);

    s_lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(s_lvgl_mux);
    xTaskCreate(lvgl_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
}
