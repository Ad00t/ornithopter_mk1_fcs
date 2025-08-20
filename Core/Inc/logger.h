/*
 * logger.h
 *
 *  Created on: Aug 19, 2025
 *      Author: adhit
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "queue.h"
#include <stdio.h>
#include <stdarg.h>

#define MAX_LOG_LEN				1024
#define LOG_LEVEL				LL_INFO
#define LOG_PREFIX_FMT			"%.3fs -- [%s] "

DEFINE_QUEUE(LogQueue, char*, 128)

typedef enum {
    LL_ERROR = 0,
    LL_WARN,
    LL_INFO,
    LL_DEBUG,
    LL_TRACE
} LogLevel;

typedef struct {
	LogQueue* q;
	FILE* out;
} Logger;

extern Logger dfl_logger;

void Logger_init(Logger* logger, FILE* out);
void Logger_log_impl(Logger* logger, LogLevel level, const char* fmt, va_list args);
void Logger_write_log(Logger* logger);

void log_error(Logger* logger, const char *fmt, ...);
void log_warn(Logger* logger, const char *fmt, ...);
void log_info(Logger* logger, const char *fmt, ...);
void log_debug(Logger* logger, const char *fmt, ...);
void log_trace(Logger* logger, const char *fmt, ...);

#endif /* LOGGER_H */
