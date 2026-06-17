/**
 ******************************************************************************
 * @file    bsp_log.h
 * @author  milFOC Team
 * @brief   Log system header - Non-blocking DMA-based logging.
 *          Provides LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR macros.
 *
 * @note    Uses USART1 for log output (DMA mode preferred).
 *          DO NOT use in high-priority ISR (JEOC current loop)!
 *          Buffer size: 256 bytes per message.
 ******************************************************************************
 */

#ifndef BSP_LOG_H
#define BSP_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include "bsp_usart.h"

#define LOG_BUFFER_SIZE 256   /* Max log message length */

/**
 * @brief Log severity levels
 */
typedef enum
{
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
} LOG_LEVEL;

/* --- Public API --- */

/**
 * @brief  Initialize the log system with a UART handle
 * @param  log_config: pointer to UART_HandleTypeDef (e.g. &huart1)
 */
void LogInit(UART_HandleTypeDef *log_config);

/**
 * @brief  Core log output function (internal use, prefer macros)
 */
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file,
               int line, const char *func, ...);

/* --- Convenience macros --- */
#define LOGDEBUG(fmt, ...)   LOG_PROTO(fmt, LOG_DEBUG,   __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGINFO(fmt, ...)    LOG_PROTO(fmt, LOG_INFO,    __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGWARNING(fmt, ...) LOG_PROTO(fmt, LOG_WARNING, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGERROR(fmt, ...)   LOG_PROTO(fmt, LOG_ERROR,   __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#endif /* BSP_LOG_H */
