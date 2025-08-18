/*
 * utils.c
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#include "utils.h"
#include <stdio.h>
#include <stdarg.h>

const char* log_level_labels[] = {
    "ERROR", "WARN", "INFO", "DEBUG", "TRACE"
};

const char* prefix_fmt = "%.3fs -- [%s] ";

void LOG(LogLevel level, const char *fmt, ...) {
	if (level > LOG_LEVEL) return;

    float t = HAL_GetTick() / 1000.0f;
    va_list args;
    va_start(args, fmt);

    printf(prefix_fmt, t, log_level_labels[level]);
    vprintf(fmt, args);
    va_end(args);
}

void LOGLN(LogLevel level, const char *fmt, ...) {
	if (level > LOG_LEVEL) return;

    float t = HAL_GetTick() / 1000.0f;
    va_list args;
    va_start(args, fmt);

    printf(prefix_fmt, t, log_level_labels[level]);
    vprintf(fmt, args);
    va_end(args);

    printf("\r\n");
}

uint8_t HEX_TO_BCD(uint8_t hex) {
	 return (((hex / 10) << 4) | (hex % 10));
}

