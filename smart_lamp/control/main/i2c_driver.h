#pragma once

#include <stdint.h>

// Estas funciones deben estar definidas en tu I2C_Driver.c
void I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
void I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
