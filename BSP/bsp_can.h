/**
 ******************************************************************************
 * @file    bsp_can.h
 * @author  milFOC Team
 * @brief   FDCAN Board Support Package for STM32G431.
 *          Provides low-level CAN send/receive/init using HAL FDCAN driver.
 *
 * @note    Hardware: FDCAN1 on PB8 (RX) / PB9 (TX)
 *          Tested with standard 11-bit ID, classic CAN (non-FD) frames, 8-byte payload.
 ******************************************************************************
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "fdcan.h"
#include "stdint.h"

/* --- Public API --- */

/**
 * @brief  Initialize CAN peripheral: configure filter, start, enable RX interrupt.
 */
void bsp_can_init(void);

/**
 * @brief  Configure FDCAN filter (mask mode, accept all standard IDs to FIFO0).
 */
void can_filter_init(void);

/**
 * @brief  Send a CAN data frame.
 * @param  hfdcan : FDCAN handle pointer
 * @param  id     : 11-bit standard CAN ID
 * @param  data   : pointer to payload data
 * @param  len    : payload length (1-8 bytes)
 * @return 0 on success, 1 on failure
 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

/**
 * @brief  Receive a CAN data frame from FIFO0.
 * @param  hfdcan : FDCAN handle pointer
 * @param  rec_id : pointer to store received CAN ID
 * @param  buf    : buffer to store received payload
 * @return received data length (bytes), 0 if no message
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);

/**
 * @brief  FDCAN1 RX callback (called from HAL_FDCAN_RxFifo0Callback).
 */
void fdcan1_rx_callback(void);

/* --- Global RX buffer (read by upper layer) --- */
extern uint8_t rx_data1[8];
extern uint16_t rec_id1;

#endif /* BSP_CAN_H */
