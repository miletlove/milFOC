/**
 ******************************************************************************
 * @file    led.h
 * @author  milFOC Team
 * @brief   RGB LED status indicator module.
 *          Uses a single WS2812B-compatible RGB LED (PC13 on milFOC board)
 *          to indicate motor state:
 *          - Green: Normal operation
 *          - Red: Fault
 *          - Blue: Calibrating
 *          - Yellow: Idle
 *
 * @note    Currently supports 1 LED on PC13.
 *          For WS2812B, uses TIM PWM+DMA for precise timing.
 ******************************************************************************
 */

#ifndef LED_H
#define LED_H

#include "general_def.h"

/* RGB Color structure */
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_TypeDef;

/* Predefined colors */
extern const RGB_Color_TypeDef LED_RED;
extern const RGB_Color_TypeDef LED_GREEN;
extern const RGB_Color_TypeDef LED_BLUE;
extern const RGB_Color_TypeDef LED_YELLOW;
extern const RGB_Color_TypeDef LED_WHITE;
extern const RGB_Color_TypeDef LED_BLACK;

#define LED_MAX_NUM 1

void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color);
void RGB_DisplayColor(RGB_Color_TypeDef color);
void RGB_DisplayColorById(uint8_t color_id);

#endif /* LED_H */
