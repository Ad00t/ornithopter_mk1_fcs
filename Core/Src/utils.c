/*
  *utils.c
 *
  * Created on: Aug 17, 2025
  *     Author: adhit
 */

#include "utils.h"
#include "logger.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <math.h>

uint8_t hex_to_bcd(uint8_t hex) {
	return (((hex / 10) << 4) | (hex % 10));
}

float clampf(float v, float lower, float upper) {
	return fmaxf(fminf(v, upper), lower);
}

uint8_t get_crc(uint8_t *message, uint8_t length) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < length; i++) {
		crc ^= message[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 1) crc ^= 0x91;
			crc >>= 1;
		}
	}
	return crc;
}

void i2c_probe(I2C_HandleTypeDef *hi2c) {
    for (uint8_t a = 0x08; a <= 0x77; a++) {                 // 7-bit
        if (HAL_I2C_IsDeviceReady(hi2c, a << 1, 2, 10) == HAL_OK) {
            log_info(&dfl_logger, "Found I2C device at 0x%02X \r\n", a);
        }
    }
}

void delay(uint32_t ms) {
    if (osKernelGetState() == osKernelRunning) {
        osDelay(ms);
        return;
    }
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {}
}
