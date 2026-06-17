/**
 ******************************************************************************
 * @file    led.c
 * @author  milFOC Team
 * @brief   RGB LED implementation.
 * @note    Uses GPIO toggle for simple LED on PC13 (single-color).
 *          TODO: Implement WS2812B timing via TIM PWM+DMA for full RGB.
 ******************************************************************************
 */

#include "led.h"
#include "gpio.h"

const RGB_Color_TypeDef LED_RED   = {255, 0, 0};
const RGB_Color_TypeDef LED_GREEN = {0, 255, 0};
const RGB_Color_TypeDef LED_BLUE  = {0, 0, 255};
const RGB_Color_TypeDef LED_YELLOW = {255, 255, 0};
const RGB_Color_TypeDef LED_WHITE = {255, 255, 255};
const RGB_Color_TypeDef LED_BLACK = {0, 0, 0};

void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color)
{
    /* Simple GPIO control for single LED on PC13 */
    (void)LedId;
    if (Color.R > 0 || Color.G > 0 || Color.B > 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  /* Active low LED on */
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    /* LED off */
    }
}

void RGB_DisplayColor(RGB_Color_TypeDef color)
{
    RGB_SetColor(0, color);
}

/**
 * @brief  Display color by ID
 *         0=RED, 1=GREEN, 2=BLUE, 3=YELLOW, 4=WHITE, 5=BLACK
 */
void RGB_DisplayColorById(uint8_t color_id)
{
    RGB_Color_TypeDef color = LED_BLACK;
    switch (color_id)
    {
    case 0: color = LED_RED;    break;
    case 1: color = LED_GREEN;  break;
    case 2: color = LED_BLUE;   break;
    case 3: color = LED_YELLOW; break;
    case 4: color = LED_WHITE;  break;
    default: break;
    }
    RGB_DisplayColor(color);
}
