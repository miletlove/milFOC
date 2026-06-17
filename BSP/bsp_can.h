/**
 ******************************************************************************
 * @file    bsp_can.h
 * @author  milFOC Team
 * @brief   CAN/FDCAN Board Support Package for STM32G431.
 *          Supports multi-instance FDCAN registration with timeout management.
 *
 * @note    Hardware: FDCAN1 on PB8 (RX) / PB9 (TX)
 *          Max registered instances: 6
 ******************************************************************************
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "general_def.h"
#include "main.h"
#include "fdcan.h"

/* CAN configuration constants */
#define FDCAN_MX_REGISTER_CNT 6   /* Max number of CAN instances */
#define DEVICE_CAN_CNT 1          /* Physical CAN peripheral count on this board */

/**
 * @brief FDCAN instance structure (ownership model)
 */
typedef struct fdcaninstance
{
    FDCAN_HandleTypeDef *fdcan_handle;  /* CAN peripheral handle */
    FDCAN_TxHeaderTypeDef txconf;       /* TX header configuration */
    uint32_t tx_id;                     /* TX message ID */
    uint32_t tx_mailbox;                /* TX mailbox slot */
    uint8_t tx_buff[8];                 /* TX buffer (max 8 bytes) */
    uint8_t rx_buff[8];                 /* RX buffer (max 8 bytes) */
    uint32_t rx_id;                     /* RX filter ID */
    uint8_t rx_len;                     /* Received data length */

    /* Module callback on received data */
    void (*fdcan_module_callback)(struct fdcaninstance *);
    void *id;                           /* Owning module pointer (cast to void*) */
} FDCANInstance;

/**
 * @brief FDCAN instance initialization configuration
 */
typedef struct
{
    FDCAN_HandleTypeDef *fdcan_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*fdcan_module_callback)(FDCANInstance *);
    void *id;                           /* Owning module pointer */
} FDCAN_Init_Config_s;

/* --- Public API --- */

/**
 * @brief  Register (initialize) a CAN instance for a module
 * @param  config: pointer to init configuration
 * @return Pointer to the registered FDCANInstance
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config);

/**
 * @brief  Set TX data length code for a CAN instance
 * @param  _instance: target CAN instance
 * @param  length: data length (0-8 bytes)
 */
void FDCANSetDLC(FDCANInstance *_instance, uint8_t length);

/**
 * @brief  Transmit message via CAN instance
 * @param  _instance: target CAN instance (write to tx_buff before calling)
 * @param  timeout: transmission timeout in milliseconds
 * @return 0 on success, non-zero on failure
 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout);

#endif /* BSP_CAN_H */
