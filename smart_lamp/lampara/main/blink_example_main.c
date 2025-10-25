#include <stdio.h>
#include <string.h> 
#include <time.h>   
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" 
#include "freertos/queue.h" 
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

// --- Inclusiones para Wi-Fi, SNTP y Servidor HTTP ---
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h" 
#include "esp_sntp.h"
#include "esp_http_server.h" // <--- ¡NUEVO! Para el servidor web

// --- Inclusión para DHT22 ---
#include "dht.h"


static const char *TAG = "MAQUINA_ESTADOS_DHT22";

// --- Pines ---
#define RGB_LED_GPIO CONFIG_BLINK_GPIO 
#define BOOT_BUTTON_GPIO 0
#define SOUND_SENSOR_GPIO 4 
#define DHT22_GPIO 6 

// --- Globales ---
static led_strip_handle_t led_strip;    
static EventGroupHandle_t state_events_group; 
static QueueHandle_t sound_events_queue; 
static httpd_handle_t server = NULL; // <--- ¡NUEVO! Handle del servidor

// --- Definición de Bits de Eventos ---
#define BUTTON_PRESSED_BIT (1 << 0) 
#define WIFI_CONNECTED_BIT (1 << 1) 
#define SOUND_DETECTED_BIT (1 << 2) 
#define REMOTE_COMMAND_BIT (1 << 3) // <--- ¡NUEVO! Para el comando Wi-Fi

// --- Definición de Estados ---
typedef enum {
    STATE_OFF,    
    STATE_RED,    
    STATE_GREEN,  // <<< ¡Este es el estado que nos interesa!
    STATE_BLUE,   
    STATE_WIFI_OK, 
    STATE_SOUND_DETECTED 
} state_t;

// Variable global para el estado
static volatile state_t current_state = STATE_OFF;


/* * Configuración del LED RGB (sin cambios) */
static void configure_led(void)
{
    ESP_LOGI(TAG, "Configurando el LED RGB en GPIO %d", RGB_LED_GPIO);
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

/* * Configuración del Botón (sin cambios) */
static void configure_button(void)
{
    ESP_LOGI(TAG, "Configurando el Botón BOOT en GPIO %d", BOOT_BUTTON_GPIO);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

/* * Configuración del Sensor de Sonido (sin cambios) */
static void configure_sound_sensor(void)
{
    ESP_LOGI(TAG, "Configurando el Sensor de Sonido en GPIO %d", SOUND_SENSOR_GPIO);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SOUND_SENSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE 
    };
    gpio_config(&io_conf);

    sound_events_queue = xQueueCreate(10, sizeof(uint32_t));
}


/* * ISR del Sensor de Sonido (sin cambios) */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(sound_events_queue, &gpio_num, NULL);
}

/* * TAREA 0: Manejador de eventos de Sonido (sin cambios) */
static void sound_handler_task(void* arg)
{
    uint32_t io_num;
    ESP_LOGI(TAG, "Tarea de Sonido iniciada.");

    for(;;) {
        if(xQueueReceive(sound_events_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "¡Sonido DETECTADO en GPIO %" PRIu32 "!", io_num);
            xEventGroupSetBits(state_events_group, SOUND_DETECTED_BIT);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            xQueueReset(sound_events_queue);
        }
    }
}

/* * TAREA 1: Lector del Botón (sin cambios) */
void button_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Tarea de botón iniciada.");
    int last_state = 1; 

    while(1) {
        int current_state_btn = gpio_get_level(BOOT_BUTTON_GPIO);
        if (last_state == 1 && current_state_btn == 0) {
            ESP_LOGI(TAG, "Botón presionado!");
            vTaskDelay(50 / portTICK_PERIOD_MS); 
            if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
                xEventGroupSetBits(state_events_group, BUTTON_PRESSED_BIT);
            }
        }
        last_state = current_state_btn;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/* * TAREA 2: Máquina de Estados (¡MODIFICADA!) */
void state_machine_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Tarea de máquina de estados iniciada.");

    while(1) {
        // --- 1. ACCIÓN DEL ESTADO (Actualizar LED) ---
        switch (current_state) {
            case STATE_OFF: led_strip_clear(led_strip); break;
            case STATE_RED: led_strip_set_pixel(led_strip, 0, 50, 0, 0); break;
            case STATE_GREEN: led_strip_set_pixel(led_strip, 0, 0, 50, 0); break;
            case STATE_BLUE: led_strip_set_pixel(led_strip, 0, 0, 0, 50); break;
            case STATE_WIFI_OK: led_strip_set_pixel(led_strip, 0, 50, 50, 50); break;
            case STATE_SOUND_DETECTED: led_strip_set_pixel(led_strip, 0, 60, 30, 0); break;
        }
        led_strip_refresh(led_strip);

        // --- 2. ESPERA DEL EVENTO ---
        ESP_LOGI(TAG, "Estado actual: %d. Esperando evento...", (int)current_state);
        
        EventBits_t bits = xEventGroupWaitBits(
            state_events_group,   
            // ¡Ahora también espera por el comando remoto!
            BUTTON_PRESSED_BIT | WIFI_CONNECTED_BIT | SOUND_DETECTED_BIT | REMOTE_COMMAND_BIT, 
            pdTRUE, pdFALSE, portMAX_DELAY   
        );

        // --- 3. TRANSICIÓN DE ESTADO ---
        if (bits & SOUND_DETECTED_BIT) {
            ESP_LOGI(TAG, "Evento: Sonido detectado.");
            current_state = STATE_SOUND_DETECTED;
        }
        // ¡El botón local O el comando remoto hacen lo mismo!
        else if (bits & BUTTON_PRESSED_BIT || bits & REMOTE_COMMAND_BIT) 
        {
            if (bits & BUTTON_PRESSED_BIT) {
                 ESP_LOGI(TAG, "Evento: Botón presionado.");
            } else {
                 ESP_LOGI(TAG, "Evento: Comando remoto '/next-state' recibido.");
            }
            
            switch (current_state) {
                case STATE_OFF: current_state = STATE_RED; break;
                case STATE_RED: current_state = STATE_GREEN; break;
                case STATE_GREEN: current_state = STATE_BLUE; break;
                case STATE_BLUE:
                case STATE_WIFI_OK: 
                case STATE_SOUND_DETECTED: 
                    current_state = STATE_OFF; break;
            }
        }
        else if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Evento: Wi-Fi y Hora OK.");
            current_state = STATE_WIFI_OK;
            time_t now;
            char strftime_buf[64];
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "Hora actual: %s", strftime_buf);
        }
    }
}

/* * TAREA 4: Lector de Temp/Hum (sin cambios) */
void temp_humidity_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Tarea de Temp/Hum (DHT22) iniciada.");
    float temperature, humidity;
    
    while(1) {
        
        if (current_state == STATE_GREEN) 
        {
            esp_err_t ret = dht_read_float_data(
                DHT_TYPE_AM2301,    // Tipo de sensor
                DHT22_GPIO,         // Pin de datos
                &humidity,          // Puntero a la variable de humedad
                &temperature        // Puntero a la variable de temperatura
            );
            
            if (ret == ESP_OK) {
                ESP_LOGI("SENSOR_TEMP", "Temperatura: %.1f C, Humedad: %.1f %%", temperature, humidity);
            } else {
                ESP_LOGE("SENSOR_TEMP", "Fallo al leer del DHT22: %s", esp_err_to_name(ret));
            }
            vTaskDelay(5000 / portTICK_PERIOD_MS);

        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}


// --- ¡NUEVO! Sección del Servidor HTTP ---

/*
 * Handler (manejador) para la URL "/next-state"
 * Esto se ejecuta cuando el otro ESP32 nos contacta.
 */
static esp_err_t next_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP handler: /next-state disparado");
    
    // ¡Disparamos el bit de evento que la máquina de estados está esperando!
    xEventGroupSetBits(state_events_group, REMOTE_COMMAND_BIT);

    // Respondemos al cliente (el otro ESP32) que todo salió bien.
    const char* resp_str = "Comando 'next-state' recibido por el Receptor.";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// Función para iniciar el servidor web
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server_handle = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true; 

    ESP_LOGI(TAG, "Iniciando servidor HTTP en puerto %d", config.server_port);
    if (httpd_start(&server_handle, &config) == ESP_OK) {
        
        httpd_uri_t next_state_uri = {
            .uri      = "/next-state", // La URL que disparará el evento
            .method   = HTTP_GET,      // Método GET
            .handler  = next_state_handler, // La función que se ejecutará
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &next_state_uri);
        return server_handle;
    }

    ESP_LOGI(TAG, "Error al iniciar el servidor HTTP");
    return NULL;
}

// Función para detener el servidor web
static void stop_webserver(httpd_handle_t server_handle)
{
    if (server_handle) {
        httpd_stop(server_handle);
    }
}
// --- Fin de la sección del Servidor HTTP ---


// --- TAREA 3: Lógica de Wi-Fi y SNTP (¡MODIFICADA!) ---
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "¡Notificación de sincronización de hora recibida!");
    xEventGroupSetBits(state_events_group, WIFI_CONNECTED_BIT);
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Inicializando SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org"); 
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); 
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Fallo al conectar. Reintentando...");
        
        // ¡Detener el servidor si nos desconectamos!
        if (server) {
            ESP_LOGI(TAG, "Deteniendo el servidor HTTP por desconexión de Wi-Fi.");
            stop_webserver(server);
            server = NULL;
        }
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "¡IP obtenida!: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // ¡Iniciar el servidor HTTP cuando tengamos IP!
        if (server == NULL) {
            ESP_LOGI(TAG, "Iniciando el servidor HTTP...");
            server = start_webserver();
        }
        
        initialize_sntp();
    }
}

void wifi_init_sta(void)
{
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
            // !! Asegúrate que estas credenciales sean las de tu red !!
            .ssid = "WiFi_Mesh-190685",
            .password = "c9KsQYCy",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finalizado. Esperando conexión...");
}
// --- Fin de la sección Wi-Fi ---


/* * Función principal (app_main) */
void app_main(void)
{
    // 0. Inicialización de NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Crear el Grupo de Eventos
    state_events_group = xEventGroupCreate();

    // 2. Configurar periféricos
    configure_led();
    configure_button();
    configure_sound_sensor(); 

    // 3. Instalar el servicio de ISR de GPIO
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(SOUND_SENSOR_GPIO, gpio_isr_handler, (void*) SOUND_SENSOR_GPIO);


    // 4. Iniciar la lógica de Wi-Fi (que ahora también inicia el servidor)
    wifi_init_sta();

    // 5. Crear las tareas
    xTaskCreate(sound_handler_task, "sound_handler", 4096, NULL, 10, NULL); 
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(state_machine_task, "state_machine", 4096, NULL, 5, NULL); 
    xTaskCreate(temp_humidity_task, "temp_task", 4096, NULL, 4, NULL); 

    ESP_LOGI(TAG, "Inicialización completada. app_main termina.");
}