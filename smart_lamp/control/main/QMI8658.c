#include "QMI8658.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

IMUdata Accel;
IMUdata Gyro;

uint8_t Device_addr ; // default for SD0/SA0 low, 0x6A if high
acc_scale_t acc_scale = ACC_RANGE_4G;
gyro_scale_t gyro_scale = GYR_RANGE_64DPS;
acc_odr_t acc_odr = acc_odr_norm_8000;
gyro_odr_t gyro_odr = gyro_odr_norm_8000;
sensor_state_t sensor_state = sensor_default;
lpf_t acc_lpf;

float accelScales, gyroScales;
float accelScales = 0;
uint8_t readings[12];
uint32_t reading_timestamp_us; // timestamp in arduino micros() time
/**
 * Inialize Wire and send default configs
 * @param addr I2C address of sensor, typically 0x6A or 0x6B
 */
void QMI8658_Init(void)
{
    uint8_t buf[1];

    // CORREGIDO: Usa la dirección que tu escaneo confirmó
    Device_addr = QMI8658_H_SLAVE_ADDRESS; // 0x6A

    I2C_Read(Device_addr, QMI8658_REVISION_ID, buf, 1);
    printf("QMI8658 Device ID: 0x%02X\n", buf[0]); // Esperado: 0x1A

    setState(sensor_running); // ⚠️ Debe llamarse antes que cualquier set*

    uint8_t ctrl7 = QMI8658_receive(QMI8658_CTRL7);
    printf("CTRL7 después de setState: 0x%02X (esperado: 0x43 o 0x83)\n", ctrl7);

    // Configuración de acelerómetro
    setAccScale(ACC_RANGE_4G);              // ±4G
    setAccODR(acc_odr_norm_250);            // 250 Hz
    setAccLPF(LPF_MODE_2);

    // Configuración de giroscopio
    setGyroScale(GYR_RANGE_64DPS);          // ±64°/s
    setGyroODR(gyro_odr_norm_250);          // 250 Hz
    setGyroLPF(LPF_MODE_2);

    // Escalas físicas
    accelScales = 4.0f / 32768.0f;          // Por la escala seleccionada
    gyroScales = 64.0f / 32768.0f;
}

void QMI8658_Loop(void)
{
  getAccelerometer();
}

/**
 * Transmit one uint8_t of data to QMI8658.
 * @param addr address of data to be written
 * @param data the data to be written
 */
void QMI8658_transmit(uint8_t addr, uint8_t data)
{
    I2C_Write(Device_addr, addr, &data, 1);
}

/**
 * Receive one uint8_t of data from QMI8658.
 * @param addr address of data to be read
 * @return the uint8_t of data that was read
 */
uint8_t QMI8658_receive(uint8_t addr)
{
    uint8_t retval;
    I2C_Read(Device_addr, addr, &retval, 1);
    return retval;
}

/**
 * Writes data to CTRL9 (command register) and waits for ACK.
 * @param command the command to be executed
 */
void QMI8658_CTRL9_Write(uint8_t command)
{
    // transmit command
    QMI8658_transmit(QMI8658_CTRL9, command);

    // wait for command to be done
    while (((QMI8658_receive(QMI8658_STATUSINT)) & 0x80) == 0x00);
}

/**
 * Set output data rate (ODR) of accelerometer.
 * @param odr acc_odr_t variable representing new data rate
 */
void setAccODR(acc_odr_t odr)
{
    if (sensor_state != sensor_default)                     // If the device is not in the default state
    {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_AODR_MASK;                        // clear previous setting
        ctrl2 |= odr;                                       // OR in new setting
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_odr = odr;
}

/**
 * Set output data rate (ODR) of gyro.
 * @param odr gyro_odr_t variable representing new data rate
 */
void setGyroODR(gyro_odr_t odr)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
    ctrl3 &= ~QMI8658_GODR_MASK; // clear previous setting
    ctrl3 |= odr; // OR in new setting
    QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_odr = odr;
}

/**
 * Set scale of accelerometer output.
 * @param scale acc_scale_t variable representing new scale
 */
void setAccScale(acc_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
    ctrl2 &= ~QMI8658_ASCALE_MASK; // clear previous setting
    ctrl2 |= scale << QMI8658_ASCALE_OFFSET; // OR in new setting
    QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_scale = scale;
}

/**
 * Set scale of gyro output.
 * @param scale gyro_scale_t variable representing new scale
 */
void setGyroScale(gyro_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
    ctrl3 &= ~QMI8658_GSCALE_MASK; // clear previous setting
    ctrl3 |= scale << QMI8658_GSCALE_OFFSET; // OR in new setting
    QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_scale = scale;
}

/**
 * Set new low-pass filter value for accelerometer
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setAccLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
    ctrl5 &= !QMI8658_ALPF_MASK;
    ctrl5 |= lpf << QMI8658_ALPF_OFFSET;
    ctrl5 |= 0x01; // turn on acc low pass filter
    QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
    acc_lpf = lpf;
}

/**
 * Set new low-pass filter value for gyro
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setGyroLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
    uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
    ctrl5 &= !QMI8658_GLPF_MASK;
    ctrl5 |= lpf << QMI8658_GLPF_OFFSET;
    ctrl5 |= 0x10; // turn on gyro low pass filter
    QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
}

/**
 * Set new state of QMI8658.
 * @param state new state to transition to
 */
void setState(sensor_state_t state)
{
    uint8_t ctrl1;
    switch (state)
    {
    case sensor_running:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        ctrl1 &= 0xFE; // enable 2MHz osc
        ctrl1 |= 0x40; // auto address increment
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        // Activar acelerómetro y giroscopio
        //QMI8658_transmit(QMI8658_CTRL7, 0x43); // <-- fuerza modo running

        // Reescribe manualmente el valor correcto en CTRL7
        QMI8658_transmit(QMI8658_CTRL7, 0x43);
        vTaskDelay(pdMS_TO_TICKS(10)); // pequeña espera para aplicar configuración

        uint8_t verify_ctrl7 = QMI8658_receive(QMI8658_CTRL7);
        printf("Verificación CTRL7 post-set: 0x%02X (esperado: 0x43)\n", verify_ctrl7);

        // Desactiva AttitudeEngine
        QMI8658_transmit(QMI8658_CTRL6, 0x00);
        break;

    case sensor_power_down:
        QMI8658_transmit(QMI8658_CTRL7, 0x00);
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        ctrl1 |= 0x01; // apagar 2MHz osc
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);
        break;

    case sensor_locking:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        ctrl1 &= 0xFE;
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        QMI8658_transmit(QMI8658_CTRL7, 0x83); // modo locking
        QMI8658_transmit(QMI8658_CTRL6, 0x00);

        QMI8658_transmit(QMI8658_CAL1_L, 0x01);
        QMI8658_CTRL9_Write(0x12);
        QMI8658_transmit(QMI8658_CAL1_L, 0x00);
        QMI8658_CTRL9_Write(0x12);
        break;

    default:
        break;
    }

    sensor_state = state;
}


void getAccelerometer(void)
{

    uint8_t buf[6];
    I2C_Read(Device_addr, QMI8658_AX_L, buf, 6);
    Accel.x = (float)((int16_t)((buf[1]<<8) | (buf[0])));
    Accel.y = (float)((int16_t)((buf[3]<<8) | (buf[2])));
    Accel.z = (float)((int16_t)((buf[5]<<8) | (buf[4])));
    Accel.x = Accel.x * accelScales;
    Accel.y = Accel.y * accelScales;
    Accel.z = Accel.z * accelScales;

}
void getGyroscope(void)
{
    uint8_t buf[6];
    I2C_Read(Device_addr, QMI8658_GX_L, buf, 6);
    Gyro.x = (float)((int16_t)((buf[1]<<8) | (buf[0])));
    Gyro.y = (float)((int16_t)((buf[3]<<8) | (buf[2])));
    Gyro.z = (float)((int16_t)((buf[5]<<8) | (buf[4])));
    Gyro.x = Gyro.x * gyroScales;
    Gyro.y = Gyro.y * gyroScales;
    Gyro.z = Gyro.z * gyroScales;
}






void QMI8658_sensor_update()
{
    // Escala del acelerómetro (ajustar según sea necesario)
    const float accelScales = 0.001f; // Ejemplo: escala en g/LSB

    // Escala del giroscopio (ajustar según sea necesario)
    const float gyroScales = 0.01f; // Ejemplo: escala en dps/LSB

    // Los registros de AX_L a GZ_H son contiguos (0x35 a 0x40), total 12 bytes
    uint8_t buf[12];
    I2C_Read(Device_addr, QMI8658_AX_L, buf, 12);

    // Acelerómetro
    Accel.x = (int16_t)((buf[1] << 8) | buf[0]) * accelScales;
    Accel.y = (int16_t)((buf[3] << 8) | buf[2]) * accelScales;
    Accel.z = (int16_t)((buf[5] << 8) | buf[4]) * accelScales;

    // Giroscopio
    Gyro.x = (int16_t)((buf[7] << 8) | buf[6]) * gyroScales;
    Gyro.y = (int16_t)((buf[9] << 8) | buf[8]) * gyroScales;
    Gyro.z = (int16_t)((buf[11] << 8) | buf[10]) * gyroScales;
}











