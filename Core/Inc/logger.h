/*
  *logger.h
 *
  * Created on: Aug 19, 2025
  *     Author: adhit
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>

#define LOG_LEVEL				LL_INFO
#define LOG_PREFIX_FMT			"%.3fs -- [%s] "

typedef enum {
    LL_TRACE = 0,
	LL_DEBUG,
    LL_INFO,
    LL_WARN,
    LL_ERROR
} LogLevel;

typedef struct {
	osMessageQueueId_t mq_id;
	size_t msg_size;
	FILE *out;
} Logger;

extern Logger dfl_logger;

void Logger_init(Logger *logger, osMessageQueueId_t q, FILE *out);
void Logger_log_impl(Logger *logger, LogLevel level, const char *fmt, va_list args);
void Logger_write_log(Logger *logger);

void log_error(Logger *logger, const char *fmt, ...);
void log_warn(Logger *logger, const char *fmt, ...);
void log_info(Logger *logger, const char *fmt, ...);
void log_debug(Logger *logger, const char *fmt, ...);
void log_trace(Logger *logger, const char *fmt, ...);

#endif /* LOGGER_H */
