/**
 ******************************************************************************
 * @file    bsp_log.c
 * @author  milFOC Team
 * @brief   Log system implementation.
 *          Formats log messages with [LEVEL] <file> | <line> | <func>: msg
 *          and sends via USART DMA for non-blocking operation.
 *
 * @note    Adapted from FalconFoc BSP/log module.
 ******************************************************************************
 */

#include "bsp_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static USARTInstance *log_usart_instance;

/**
 * @brief  Initialize log system with specified UART
 */
void LogInit(UART_HandleTypeDef *log_config)
{
    USART_Init_Config_s config;
    config.usart_handle = log_config;
    log_usart_instance = USARTRegister(&config);
}

/**
 * @brief  Core log formatting and output function
 */
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file,
               int line, const char *func, ...)
{
    char tmp[LOG_BUFFER_SIZE];
    char buf[LOG_BUFFER_SIZE];
    memset(tmp, 0, sizeof(tmp));
    memset(buf, 0, sizeof(buf));

    va_list args;
    va_start(args, func);
    vsnprintf(tmp, sizeof(tmp) - 1, fmt, args);
    va_end(args);

    switch (level)
    {
    case LOG_DEBUG:
        snprintf(buf, sizeof(buf), "[DEBUG] <%s:%d> %s(): %s\r\n", file, line, func, tmp);
        break;
    case LOG_INFO:
        snprintf(buf, sizeof(buf), "[INFO ] <%s:%d> %s(): %s\r\n", file, line, func, tmp);
        break;
    case LOG_WARNING:
        snprintf(buf, sizeof(buf), "[WARN ] <%s:%d> %s(): %s\r\n", file, line, func, tmp);
        break;
    case LOG_ERROR:
        snprintf(buf, sizeof(buf), "[ERROR] <%s:%d> %s(): %s\r\n", file, line, func, tmp);
        break;
    default:
        return;
    }

    /* Send via USART DMA for non-blocking output */
    if (log_usart_instance != NULL)
    {
        USARTSend(log_usart_instance, (uint8_t *)buf, strlen(buf), USART_TRANSFER_DMA);
    }
}
