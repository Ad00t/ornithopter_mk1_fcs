/*
  *utils.h
 *
  * Created on: Aug 17, 2025
  *     Author: adhit
 */

#ifndef UTILS_H
#define UTILS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

uint8_t hex_to_bcd(uint8_t hex);
float clampf(float v, float min, float max);

uint8_t get_crc(uint8_t *message, uint8_t length);
void i2c_probe(I2C_HandleTypeDef *hi2c);

void delay(uint32_t ms);

#endif /* UTILS_H */
