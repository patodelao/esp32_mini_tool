/*
 * camera.c — Driver de la cámara (OV2640 del AI-Thinker ESP32-CAM).
 */
#include "camera.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "camera";

static bool s_ready = false;

/* Mapa de pines del AI-Thinker ESP32-CAM. Es el estándar de esta placa; si
 * algún día se usa otra (M5Camera, ESP-EYE…) cambia solo este bloque. */
#define PIN_PWDN   32
#define PIN_RESET  -1
#define PIN_XCLK    0
#define PIN_SIOD   26   /* SDA del bus SCCB */
#define PIN_SIOC   27   /* SCL del bus SCCB */
#define PIN_Y9     35
#define PIN_Y8     34
#define PIN_Y7     39
#define PIN_Y6     36
#define PIN_Y5     21
#define PIN_Y4     19
#define PIN_Y3     18
#define PIN_Y2      5
#define PIN_VSYNC  25
#define PIN_HREF   23
#define PIN_PCLK   22

esp_err_t camera_init(void)
{
    if (s_ready) return ESP_OK;

    /* Sin PSRAM no hay lugar para un frame buffer decente en JPEG. Se degrada
     * a una resolución chica y un solo buffer en RAM interna en vez de fallar,
     * pero lo normal en esta placa es que haya PSRAM. */
    bool psram = (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0);
    if (!psram) {
        ESP_LOGW(TAG, "Sin PSRAM: se usa resolucion reducida");
    }

    camera_config_t cfg = {
        .pin_pwdn = PIN_PWDN,
        .pin_reset = PIN_RESET,
        .pin_xclk = PIN_XCLK,
        .pin_sccb_sda = PIN_SIOD,
        .pin_sccb_scl = PIN_SIOC,
        .pin_d7 = PIN_Y9, .pin_d6 = PIN_Y8, .pin_d5 = PIN_Y7, .pin_d4 = PIN_Y6,
        .pin_d3 = PIN_Y5, .pin_d2 = PIN_Y4, .pin_d1 = PIN_Y3, .pin_d0 = PIN_Y2,
        .pin_vsync = PIN_VSYNC,
        .pin_href = PIN_HREF,
        .pin_pclk = PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = psram ? FRAMESIZE_SVGA : FRAMESIZE_QVGA,  /* 800x600 / 320x240 */
        .jpeg_quality = 12,                                       /* 0=mejor, 63=peor */
        .fb_count     = psram ? 2 : 1,
        .fb_location  = psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,   /* la foto más nueva, no la encolada */
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init: %s", esp_err_to_name(err));
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "Camara lista (%s)", psram ? "SVGA/PSRAM" : "QVGA/DRAM");
    return ESP_OK;
}

bool camera_ready(void) { return s_ready; }
