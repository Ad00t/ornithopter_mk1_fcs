/*
  *logger.c
 *
  * Created on: Aug 19, 2025
  *     Author: adhit
 */

#include "logger.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

const char *ll_labels[] = {
	"TRACE", "DEBUG", "INFO", "WARN", "ERROR"
};

void Logger_init(Logger *logger, osMessageQueueId_t mq_id, FILE *out) {
	logger->mq_id = mq_id;
	logger->msg_size = (size_t)osMessageQueueGetMsgSize(mq_id);
	logger->out = out;
}

void Logger_log_impl(Logger *logger, LogLevel level, const char *fmt, va_list args) {
    if (level < LOG_LEVEL) return;

    float t_s = 0.0f;
    if (osKernelGetState() == osKernelRunning) {
        t_s = (float)osKernelGetTickCount() / configTICK_RATE_HZ;
    }

    char buf[logger->msg_size];
    size_t max_msg_len = sizeof(buf) - 3;
    int n = snprintf(buf, max_msg_len, LOG_PREFIX_FMT, t_s, ll_labels[level]);
    if (n < 0) n = 0;
    vsnprintf(buf + n, max_msg_len - n, fmt, args);

    size_t len = strlen(buf);
    if (len + 2 < sizeof(buf)) {
        buf[len] = '\r';
        buf[len + 1] = '\n';
        buf[len + 2] = '\0';
    } else {
        buf[sizeof(buf) - 3] = '\r';
        buf[sizeof(buf) - 2] = '\n';
        buf[sizeof(buf) - 1] = '\0';
    }

    if (osKernelGetState() == osKernelRunning) {
        osMessageQueuePut(logger->mq_id, buf, (uint8_t)level, 10);
    } else {
        fprintf(logger->out, "%s", buf);
    }
}

void Logger_write_log(Logger *logger) {
	char buf[logger->msg_size];
	while (osMessageQueueGet(logger->mq_id, buf, NULL, 10) == osOK) {
		fprintf(logger->out, "%s", buf);
	}
}

void log_error(Logger *logger, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_ERROR, fmt, args);
    va_end(args);
}

void log_warn(Logger *logger, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_WARN, fmt, args);
    va_end(args);
}

void log_info(Logger *logger, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_INFO, fmt, args);
    va_end(args);
}

void log_debug(Logger *logger, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_DEBUG, fmt, args);
    va_end(args);
}

void log_trace(Logger *logger, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Logger_log_impl(logger, LL_TRACE, fmt, args);
    va_end(args);
}
