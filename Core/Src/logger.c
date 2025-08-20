/*
 * logger.c
 *
 *  Created on: Aug 19, 2025
 *      Author: adhit
 */

#include "logger.h"
#include "utils.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

const char* ll_labels[] = {
	"ERROR", "WARN", "INFO", "DEBUG", "TRACE"
};

void Logger_init(Logger* logger, FILE* out) {
	logger->out = out;
	LogQueue_Init(logger->q, string_copy, string_free);
}

void Logger_log_impl(Logger* logger, LogLevel level, const char* fmt, va_list args) {
	if (level > LOG_LEVEL) return;
    float t_s = HAL_GetTick() / 1000.0f;

    char buf[MAX_LOG_LEN];
    int n = snprintf(buf, sizeof(buf), LOG_PREFIX_FMT, t_s, ll_labels[level]);
    if (n < 0) n = 0;
    vsnprintf(buf + n, sizeof(buf) - n, fmt, args);
    if (!LogQueue_Enqueue(logger->q, buf)) {
    	printf("ERROR: LOG QUEUE FULL\r\n");
    }
}

void Logger_write_log(Logger* logger) {
	char buf[MAX_LOG_LEN];
	if (LogQueue_Dequeue(logger->q, buf)) {
		fprintf(logger->out, "%s", buf);
	}
}

void log_error(Logger* logger, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_ERROR, fmt, args);
    va_end(args);
}

void log_warn(Logger* logger, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_WARN, fmt, args);
    va_end(args);
}

void log_info(Logger* logger, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_INFO, fmt, args);
    va_end(args);
}

void log_debug(Logger* logger, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_DEBUG, fmt, args);
    va_end(args);
}

void log_trace(Logger* logger, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_TRACE, fmt, args);
    va_end(args);
}
