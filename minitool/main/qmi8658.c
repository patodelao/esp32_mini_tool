/*
 * qmi8658.c — Implementación del driver QMI8658 sobre el bus I2C del bsp.
 */
#include "qmi8658.h"
#include "bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "qmi8658";

/* Direcciones I2C posibles (según el pin SA0 de la placa) */
#define QMI8658_ADDR_HIGH 0x6B
#define QMI8658_ADDR_LOW  0x6A

/* Registros */
#define REG_WHO_AM_I 0x00 /* debe leer 0x05 */
#define REG_CTRL1    0x02 /* config serial: auto-incremento de dirección */
#define REG_CTRL2    0x03 /* accel: escala + ODR */
#define REG_CTRL3    0x04 /* gyro: escala + ODR */
#define REG_CTRL7    0x08 /* habilitación de sensores */
#define REG_RESET    0x60 /* soft reset escribiendo 0xB0 */
#define REG_AX_L     0x35 /* datos accel: 6 bytes L/H por eje X,Y,Z */
#define REG_GX_L     0x3B /* datos gyro: 6 bytes L/H por eje X,Y,Z */

#define WHO_AM_I_VAL 0x05

/* CTRL2 = aFS ±4g (001) | aODR 125Hz en modo 6DOF (0110) */
#define CTRL2_VAL 0x16
#define ACCEL_LSB_PER_G 8192.0f /* 32768 / 4g */

/* CTRL3 = gFS ±256dps (100) | gODR 125Hz (0110) */
#define CTRL3_VAL 0x46
#define GYRO_LSB_PER_DPS 128.0f /* 32768 / 256dps */

#define I2C_TIMEOUT_MS 100

static uint8_t s_addr = 0;
static bool s_available = false;

/* Con el driver nuevo el chip es un "dispositivo" colgado del bus, no una
 * dirección que se pasa en cada transacción. */
static i2c_master_dev_handle_t s_dev = NULL;

bool qmi8658_available(void) { return s_available; }

static esp_err_t reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* Registra el chip en la dirección dada, soltando el registro anterior: al
 * arrancar se prueban las dos posibles según el pin SA0. */
static esp_err_t dev_attach(uint8_t addr)
{
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = BSP_I2C_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bsp_i2c_bus(), &cfg, &s_dev);
    if (err == ESP_OK) s_addr = addr;
    return err;
}

static esp_err_t probe(uint8_t addr)
{
    esp_err_t err = dev_attach(addr);
    if (err != ESP_OK) return err;

    uint8_t who = 0;
    err = reg_read(REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) return err;
    return (who == WHO_AM_I_VAL) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t qmi8658_init(void)
{
    /* El chip puede estar en 0x6B o 0x6A según SA0 */
    if (probe(QMI8658_ADDR_HIGH) != ESP_OK && probe(QMI8658_ADDR_LOW) != ESP_OK) {
        ESP_LOGW(TAG, "QMI8658 no detectado en el bus I2C");
        return ESP_ERR_NOT_FOUND;
    }

    /* Soft reset y espera de arranque */
    ESP_ERROR_CHECK(reg_write(REG_RESET, 0xB0));
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Auto-incremento de dirección para lecturas en ráfaga, little-endian */
    ESP_ERROR_CHECK(reg_write(REG_CTRL1, 0x40));
    ESP_ERROR_CHECK(reg_write(REG_CTRL2, CTRL2_VAL));
    ESP_ERROR_CHECK(reg_write(REG_CTRL3, CTRL3_VAL));
    /* Habilitar accel + gyro */
    ESP_ERROR_CHECK(reg_write(REG_CTRL7, 0x03));

    s_available = true;
    ESP_LOGI(TAG, "QMI8658 inicializado en 0x%02X", s_addr);
    return ESP_OK;
}

/* Lee 3 ejes de 16 bits little-endian desde 'base' y los escala por 1/lsb. */
static esp_err_t read_axes(uint8_t base, float lsb, float *x, float *y, float *z)
{
    if (!s_available) return ESP_ERR_INVALID_STATE;
    uint8_t raw[6];
    esp_err_t err = reg_read(base, raw, sizeof(raw));
    if (err != ESP_OK) return err;

    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);
    *x = rx / lsb;
    *y = ry / lsb;
    *z = rz / lsb;
    return ESP_OK;
}

esp_err_t qmi8658_read_accel(float *ax, float *ay, float *az)
{
    return read_axes(REG_AX_L, ACCEL_LSB_PER_G, ax, ay, az);
}

esp_err_t qmi8658_read_gyro(float *gx, float *gy, float *gz)
{
    return read_axes(REG_GX_L, GYRO_LSB_PER_DPS, gx, gy, gz);
}
