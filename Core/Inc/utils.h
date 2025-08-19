/*
 * utils.h
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include "stm32f4xx_hal.h"

typedef enum {
    LL_ERROR = 0,
    LL_WARN,
    LL_INFO,
    LL_DEBUG,
    LL_TRACE
} LogLevel;

#define LOG_LEVEL	LL_INFO

void LOG(LogLevel level, const char *fmt, ...);
void LOGLN(LogLevel level, const char *fmt, ...);

uint8_t hex_to_bcd(uint8_t hex);
float clampf(float v, float min, float max);

#endif /* UTILS_H */
