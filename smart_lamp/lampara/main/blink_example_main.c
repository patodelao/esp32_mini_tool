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
#include "esp_http_server.h" 

// --- Inclusión para DHT22 ---
#include "dht.h"

static const char *TAG = "MAQUINA_ESTADOS_FINAL";

// --- Pines ---
#define RGB_LED_GPIO CONFIG_BLINK_GPIO 
#define BOOT_BUTTON_GPIO 0
#define SOUND_SENSOR_GPIO 4 
#define DHT22_GPIO 6 
#define RELAY_PIN GPIO_NUM_11 // <--- ¡Pin del Relé!
#define TRANSISTOR_PIN GPIO_NUM_5 // <--- ¡NUEVO! Pin para controlar el transistor

// --- Lógica del Relé (Ajusta esto si tu relé es 'active-low') ---
#define RELAY_ON_LEVEL  1 // Nivel para ENCENDER el relé (1 = ALTO)
#define RELAY_OFF_LEVEL 0 // Nivel para APAGAR el relé (0 = BAJO)

// --- Globales ---
static led_strip_handle_t led_strip;    
static EventGroupHandle_t state_events_group; 
static QueueHandle_t sound_events_queue; 
static httpd_handle_t server = NULL; 
// ¡NUEVO! Flag para indicar si la luz blanca está encendida
static volatile bool white_light_on = false;

// --- Definición de Bits de Eventos ---
#define BUTTON_PRESSED_BIT (1 << 0) 
#define WIFI_CONNECTED_BIT (1 << 1) 
#define SOUND_DETECTED_BIT (1 << 2) 
#define REMOTE_COMMAND_BIT (1 << 3) 
#define REFRESH_DISPLAY_BIT (1 << 4) // Bit para refrescar/forzar actualización del display/estado


// --- Definición de Estados ---
typedef enum {
    STATE_OFF,    
    STATE_RED,    
    STATE_GREEN,  
    STATE_BLUE,   
    STATE_WIFI_OK, 
    STATE_SOUND_DETECTED,
    STATE_RELAY_ON, // <--- ¡Nuevo estado del Relé!
    STATE_TRANSISTOR_ON
} state_t;

// Variable global para el estado (volatile para acceso seguro entre tareas)
static volatile state_t current_state = STATE_OFF;


/* * Configuración del LED RGB
 */
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

/* * Configuración del Botón
 */
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

/* * Configuración del Sensor de Sonido
 */
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

/* * ¡NUEVO! Configuración del pin del Relé
 */
static void configure_relay(void)
{
    ESP_LOGI(TAG, "Configurando el pin del Relé en GPIO %d", RELAY_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT, // Es una salida
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Asegurarse de que el relé comience APAGADO
    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL); 
}

/* * ¡NUEVO! Configuración del pin del Transistor
 */
static void configure_transistor_pin(void)
{
    ESP_LOGI(TAG, "Configurando el pin del Transistor en GPIO %d", TRANSISTOR_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRANSISTOR_PIN),
        .mode = GPIO_MODE_OUTPUT, // Es una salida
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Asegura que empiece en BAJO
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Asegurarse de que el transistor comience APAGADO (sin conducir a tierra)
    gpio_set_level(TRANSISTOR_PIN, 0);
}



/* * ISR del Sensor de Sonido
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(sound_events_queue, &gpio_num, NULL);
}

/* * TAREA 0: Manejador de eventos de Sonido
 */
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

/* * TAREA 1: Lector del Botón

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
} */

/* * TAREA 1: Lector del Botón (¡MODIFICADA para detectar presión larga!)
 */

void button_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Tarea de botón (simple) iniciada.");
    int last_state = 1;

    while(1) {
        int current_state_btn = gpio_get_level(BOOT_BUTTON_GPIO);

        // Chequeo de "flanco de bajada" (cuando pasa de 1 a 0)
        if (last_state == 1 && current_state_btn == 0) {
            ESP_LOGD(TAG, "Button pressed down");
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce inicial

            // Volvemos a leer. Si sigue en 0, es una pulsación válida
            if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
                 ESP_LOGI(TAG, "¡Presión CORTA detectada!");
                 xEventGroupSetBits(state_events_group, BUTTON_PRESSED_BIT);
                 // Esperar a que se suelte para evitar múltiples detecciones
                 while(gpio_get_level(BOOT_BUTTON_GPIO) == 0){
                     vTaskDelay(pdMS_TO_TICKS(10));
                 }
                 ESP_LOGD(TAG, "Button released");
                 current_state_btn = 1; // Actualizar estado para la próxima iteración
            }
        }

        last_state = current_state_btn;
        // Pequeña pausa para ceder CPU
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
/* * TAREA 2: Máquina de Estados (¡MODIFICADA!)
 */

/* * TAREA 2: Máquina de Estados (¡MODIFICADA para luz blanca y prioridad de estado!)
 */


/* * TAREA 2: Máquina de Estados (¡MODIFICADA para luz blanca y prioridad de estado!)
 */
/* * TAREA 2: Máquina de Estados (¡MODIFICADA para Transistor!)
 */
void state_machine_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Tarea de máquina de estados iniciada.");

    while(1) {
        // --- 1. ACCIÓN DEL ESTADO (Actualizar LED, Relé y Transistor) ---
        // ¡MODIFICADO! Controla el transistor en todos los estados.
        if (white_light_on) {
            // Modo Linterna: LED Blanco, Relé APAGADO, Transistor APAGADO
            led_strip_set_pixel(led_strip, 0, 80, 80, 80);
            gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
            gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF
            ESP_LOGD(TAG, "Acción: Modo Luz Blanca");
        } else {
            // Modo Normal: Usa el estado actual
            ESP_LOGD(TAG, "Acción: Estado Normal (%d)", (int)current_state);
            switch (current_state) {
                case STATE_OFF:
                    led_strip_clear(led_strip);
                    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
                    gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF
                    break;
                case STATE_RED:
                    led_strip_set_pixel(led_strip, 0, 50, 0, 0); // Rojo
                    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
                    gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF
                    break;
                case STATE_GREEN:
                    led_strip_set_pixel(led_strip, 0, 0, 50, 0); // Verde
                    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
                    gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF
                    break;
                case STATE_BLUE:
                    led_strip_set_pixel(led_strip, 0, 0, 0, 50); // Azul
                    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
                    gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF
                    break;
                case STATE_WIFI_OK:
                    led_strip_set_pixel(led_strip, 0, 50, 50, 50); // Blanco tenue
                    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
                    gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF
                    break;
                case STATE_SOUND_DETECTED:
                    led_strip_set_pixel(led_strip, 0, 60, 30, 0); // Naranja
                    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL);
                    gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF
                    break;
                case STATE_RELAY_ON:
                    led_strip_clear(led_strip); // LED Apagado
                    gpio_set_level(RELAY_PIN, RELAY_ON_LEVEL); // Relé Encendido
                    gpio_set_level(TRANSISTOR_PIN, 0); // <-- Transistor OFF (¿O debería estar ON aquí?)
                                                       // Depende de tu circuito exacto. Si el transistor
                                                       // controla la ALIMENTACIÓN del relé, debería estar ON (1).
                                                       // Si solo controla otra cosa, déjalo en OFF (0).
                                                       // Lo dejaré en OFF asumiendo que el relé ya tiene poder.
                    break;

                // --- ¡NUEVO ESTADO! ---
                case STATE_TRANSISTOR_ON:
                    led_strip_clear(led_strip); // LED Apagado
                    gpio_set_level(RELAY_PIN, RELAY_OFF_LEVEL); // Relé Apagado
                    gpio_set_level(TRANSISTOR_PIN, 1); // <-- Transistor ON (conduce a tierra)
                    break;
            }
        }
        led_strip_refresh(led_strip);

        // --- 2. ESPERA DEL EVENTO ---
        // (No cambia la espera)
        ESP_LOGI(TAG, "Estado actual: %d, Luz Blanca: %s. Esperando evento...",
                 (int)current_state, white_light_on ? "ON" : "OFF");

        EventBits_t bits = xEventGroupWaitBits(
            state_events_group,
            BUTTON_PRESSED_BIT | WIFI_CONNECTED_BIT | SOUND_DETECTED_BIT | REMOTE_COMMAND_BIT | REFRESH_DISPLAY_BIT,
            pdTRUE, pdFALSE, portMAX_DELAY
        );

        // --- 3. TRANSICIÓN DE ESTADO Y ACCIONES ---
        // (Modificamos el ciclo del botón/remoto)
        if (bits & REFRESH_DISPLAY_BIT) {
             ESP_LOGD(TAG, "Evento: Refresco de display solicitado.");
             // No hace nada, solo despierta la tarea
        }
        else if (bits & SOUND_DETECTED_BIT) {
            ESP_LOGI(TAG, "Evento: Sonido detectado.");
            current_state = STATE_SOUND_DETECTED;
        }
        else if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Evento: Wi-Fi y Hora OK.");
            current_state = STATE_WIFI_OK;
            // ... (imprimir hora) ...
            time_t now;
            char strftime_buf[64];
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "Hora actual: %s", strftime_buf);
        }
        else if (bits & BUTTON_PRESSED_BIT || bits & REMOTE_COMMAND_BIT)
        {
            if (bits & BUTTON_PRESSED_BIT) {
                 ESP_LOGI(TAG, "Evento: Presión CORTA del botón.");
            } else {
                 ESP_LOGI(TAG, "Evento: Comando remoto '/next-state' recibido.");
            }

            // --- ¡Ciclo manual modificado! ---
            switch (current_state) {
                case STATE_OFF: current_state = STATE_RED; break;
                case STATE_RED: current_state = STATE_GREEN; break;
                case STATE_GREEN: current_state = STATE_BLUE; break;
                case STATE_BLUE: current_state = STATE_RELAY_ON; break;
                case STATE_RELAY_ON: current_state = STATE_TRANSISTOR_ON; break; // <-- Pasa a Transistor
                
                // Si está en el último estado (Transistor) o en un estado automático, vuelve a OFF
                case STATE_TRANSISTOR_ON: // <-- Nuevo
                case STATE_WIFI_OK:
                case STATE_SOUND_DETECTED:
                    current_state = STATE_OFF; break;
            }
        }
    } // Fin del while(1)
} // Fin de la función



 /* * TAREA 4: Lector de Temp/Hum (DHT22)
 */
void temp_humidity_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Tarea de Temp/Hum (DHT22) iniciada.");
    float temperature, humidity;
    
    // (Asegúrate de tener la resistencia pull-up de 4.7k en el pin 6)
    
    while(1) {
        
        // Solo lee si el estado es VERDE
        if (current_state == STATE_GREEN) 
        {
            esp_err_t ret = dht_read_float_data(
                DHT_TYPE_AM2301, // Tipo de sensor (DHT22)
                DHT22_GPIO,     // Pin de datos (GPIO 6)
                &humidity,      
                &temperature    
            );
            
            if (ret == ESP_OK) {
                ESP_LOGI("SENSOR_TEMP", "Temperatura: %.1f C, Humedad: %.1f %%", temperature, humidity);
            } else {
                ESP_LOGE("SENSOR_TEMP", "Fallo al leer del DHT22: %s", esp_err_to_name(ret));
            }
            // El DHT22 solo se puede leer cada 2 seg. Esperamos 5.
            vTaskDelay(5000 / portTICK_PERIOD_MS);

        } else {
            // Si no es estado VERDE, solo duerme 1 seg y vuelve a chequear
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}


// --- Sección del Servidor HTTP ---

/*
 * Handler (manejador) para la URL "/next-state"
 */
static esp_err_t next_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP handler: /next-state disparado");
    
    // Dispara el bit de evento que la máquina de estados está esperando
    xEventGroupSetBits(state_events_group, REMOTE_COMMAND_BIT);

    // Responde al cliente
    const char* resp_str = "Comando 'next-state' recibido por el Receptor.";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/*
 * ¡NUEVO! Handler para encender la luz blanca ("/light/on")
 */
static esp_err_t light_on_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP handler: /light/on disparado");
    white_light_on = true; // Activa el flag global

    // Respondemos al cliente
    const char* resp_str = "Luz blanca ENCENDIDA.";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/*
 * ¡NUEVO! Handler para apagar la luz blanca ("/light/off")
 */
static esp_err_t light_off_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTP handler: /light/off disparado");
    white_light_on = false; // Desactiva el flag global

    // Respondemos al cliente
    const char* resp_str = "Luz blanca APAGADA.";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Función para iniciar el servidor web
// Función para iniciar el servidor web (¡MODIFICADA!)
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server_handle = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Iniciando servidor HTTP en puerto %d", config.server_port);
    if (httpd_start(&server_handle, &config) == ESP_OK) {

        // Handler para cambiar al siguiente estado
        httpd_uri_t next_state_uri = {
            .uri      = "/next-state",
            .method   = HTTP_GET,
            .handler  = next_state_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &next_state_uri);

        // ¡NUEVO! Handler para encender la luz
        httpd_uri_t light_on_uri = {
            .uri      = "/light/on",
            .method   = HTTP_GET,
            .handler  = light_on_handler, // <--- Nueva función
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &light_on_uri);

        // ¡NUEVO! Handler para apagar la luz
        httpd_uri_t light_off_uri = {
            .uri      = "/light/off",
            .method   = HTTP_GET,
            .handler  = light_off_handler, // <--- Nueva función
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server_handle, &light_off_uri);

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


// --- TAREA 3: Lógica de Wi-Fi y SNTP (¡Modificada!) ---
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
        
        // Detener el servidor si nos desconectamos
        if (server) {
            ESP_LOGI(TAG, "Deteniendo el servidor HTTP por desconexión de Wi-Fi.");
            stop_webserver(server);
            server = NULL;
        }
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "¡IP obtenida!: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // Iniciar el servidor HTTP cuando tengamos IP
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


/* * Función principal (app_main)
 */
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
    configure_relay(); // <--- ¡Relé añadido!
    configure_transistor_pin(); // <--- ¡AÑADIDO!

    // 3. Instalar el servicio de ISR de GPIO
    // (Tu código tenía ESP_INTR_FLAG_LEVEL3, lo he mantenido)
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