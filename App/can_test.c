/**
 ******************************************************************************
 * @file    can_test.c
 * @author  milFOC Team
 * @brief   CAN communication test — heartbeat TX + echo RX.
 *
 *          Test procedure:
 *          1. Board sends a heartbeat frame (ID=0x7FF, 8-byte counter) at ~10Hz
 *          2. On receiving any standard ID frame, echoes it back with ID+1
 *          3. PC-side tool (e.g. USB-CAN adapter) can verify bidirectional comm
 *
 *          LED indicator (PC13): toggles on each successful TX
 ******************************************************************************
 */

#include "can_test.h"
#include "bsp_fdcan.h"
#include "main.h"
#include "string.h"

/* Test CAN IDs */
#define CAN_TEST_HEARTBEAT_ID  0x7FF
#define CAN_TEST_ECHO_ID_OFFSET 1     /* echo with rx_id + 1 */

static uint32_t tx_counter = 0;
static uint32_t last_tx_tick = 0;

void CAN_Test_Init(void)
{
    /* Initialize CAN hardware (filter + start + RX interrupt) */
    bsp_can_init();
}

void CAN_Test_Task(void)
{
    /* --- Heartbeat TX --- */
    if (HAL_GetTick() - last_tx_tick > 100)  /* 10Hz */
    {
        last_tx_tick = HAL_GetTick();

        uint8_t tx_data[8];
        memset(tx_data, 0, sizeof(tx_data));
        tx_data[0] = (tx_counter >> 24) & 0xFF;
        tx_data[1] = (tx_counter >> 16) & 0xFF;
        tx_data[2] = (tx_counter >> 8)  & 0xFF;
        tx_data[3] =  tx_counter        & 0xFF;

        if (fdcanx_send_data(&hfdcan1, CAN_TEST_HEARTBEAT_ID, tx_data, 8) == 0)
        {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  /* LED flash on TX */
            tx_counter++;
        }
    }

    /* --- Echo RX frames (non-blocking check) --- */
    /* Note: rx_data1[] is filled by HAL_FDCAN_RxFifo0Callback ISR.
     *       Only echo back frames that are NOT heartbeat echoes from ourselves. */
    uint8_t  rx_buf[8];
    uint16_t rx_id;

    if (fdcanx_receive(&hfdcan1, &rx_id, rx_buf) > 0)
    {
        if (rx_id != CAN_TEST_HEARTBEAT_ID + CAN_TEST_ECHO_ID_OFFSET)
        {
            /* Echo back: send same data with ID = rx_id + 1 */
            fdcanx_send_data(&hfdcan1, rx_id + CAN_TEST_ECHO_ID_OFFSET, rx_buf, 8);
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  /* LED flash on RX echo */
        }
    }
}
