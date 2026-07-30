/*
 * dht.c — Implementación del lector DHT11/DHT22 por bit-banging.
 *
 * Protocolo (1 hilo): el host tira la línea a 0 un rato para despertar al
 * sensor, la suelta, y el sensor responde con 80us en 0 + 80us en 1 y luego 40
 * bits. Cada bit es ~50us en 0 seguido de un pulso en 1 cuyo LARGO decide el
 * valor: pulso corto (~27us) = 0, largo (~70us) = 1. En vez de un umbral fijo en
 * microsegundos (que depende del reloj y del overhead del bucle), se compara el
 * largo del pulso alto contra el del bajo previo: si el alto es más largo que el
 * bajo, es un 1. Así se autocalibra y no depende de medir us exactos.
 *
 * El timing es sensible, así que los 40 bits se leen con las interrupciones de
 * este core deshabilitadas (~5 ms). El pulso de arranque (que en el DHT11 dura
 * 18-20 ms) queda FUERA de la sección crítica para no bloquear tanto.
 */
#include "dht.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us */
#include "esp_log.h"

static const char *TAG = "dht";

/* Cuenta microsegundos mientras el pin NO está en 'level'; devuelve la duración
 * al alcanzarlo, o -1 si se pasó del timeout (el sensor no respondió). */
static int wait_level(gpio_num_t pin, int level, int timeout_us)
{
    int us = 0;
    while (gpio_get_level(pin) != level) {
        if (us++ > timeout_us) return -1;
        esp_rom_delay_us(1);
    }
    return us;
}

void dht_init(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

int dht_read(gpio_num_t pin, dht_type_t type, float *temp_c, float *hum)
{
    uint8_t data[5] = { 0 };

    /* Arranque: host a 0 (>=1 ms; el DHT11 pide ~18 ms), soltar, esperar. Fuera
     * de la sección crítica para no bloquear el core 18-20 ms. */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    esp_rom_delay_us(type == DHT_TYPE_DHT11 ? 20000 : 1200);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(40);
    gpio_set_direction(pin, GPIO_MODE_INPUT);   /* el pull-up mantiene el 1 */

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    /* stage: 0 = OK; 1-3 = el sensor no dio la respuesta inicial; 4 = se cortó a
     * mitad de los bits. No se puede loguear con las interrupciones apagadas, así
     * que se guarda la etapa y se reporta al salir de la sección crítica. */
    int stage = 0;
    if (wait_level(pin, 0, 120) < 0) stage = 1;
    else if (wait_level(pin, 1, 120) < 0) stage = 2;
    else if (wait_level(pin, 0, 120) < 0) stage = 3;
    else {
        for (int i = 0; i < 40; i++) {
            int low  = wait_level(pin, 1, 120);   /* dura el 0 de ~50us */
            int high = wait_level(pin, 0, 120);   /* dura el 1 (corto=0, largo=1) */
            if (low < 0 || high < 0) { stage = 4; break; }
            data[i / 8] <<= 1;
            if (high > low) data[i / 8] |= 1;     /* pulso alto más largo = bit 1 */
        }
    }

    portEXIT_CRITICAL(&mux);

    if (stage != 0) {
        /* El sensor no contestó (o se cortó). Casi siempre es HARDWARE: falta el
         * pull-up de 10k entre DATA y 3.3V, el cable de DATA no llega al GPIO, o
         * la alimentación. NO es el timing del código. */
        ESP_LOGW(TAG, "sin respuesta del sensor (etapa %d) -> falta pull-up 10k a "
                      "3.3V / cable DATA al GPIO / alimentacion", stage);
        return stage;                       /* 1..4 */
    }

    /* Checksum: los 4 bytes de datos suman el 5º (mod 256). */
    uint8_t sum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (sum != data[4]) {
        /* Llegaron 40 bits pero no cierran: esto SÍ apunta al timing/ruido de la
         * lectura (o cable largo). Los bytes crudos ayudan a diagnosticar. */
        ESP_LOGW(TAG, "checksum falla: %02X %02X %02X %02X chk=%02X calc=%02X (timing/ruido)",
                 data[0], data[1], data[2], data[3], data[4], sum);
        return 5;
    }

    if (type == DHT_TYPE_DHT11) {
        *hum    = (float)data[0] + (float)data[1] * 0.1f;
        *temp_c = (float)data[2] + (float)(data[3] & 0x7F) * 0.1f;
        if (data[3] & 0x80) *temp_c = -*temp_c;
    } else {   /* DHT22 / AM2302: 16 bits ×10, temperatura con signo en el bit alto */
        *hum = (float)(((uint16_t)data[0] << 8) | data[1]) * 0.1f;
        uint16_t raw = ((uint16_t)(data[2] & 0x7F) << 8) | data[3];
        *temp_c = (float)raw * 0.1f;
        if (data[2] & 0x80) *temp_c = -*temp_c;
    }
    return 0;
}
