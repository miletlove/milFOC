/**
 ******************************************************************************
 * @file    can_driver.h
 * @author  milFOC Team
 * @brief   CAN communication protocol driver.
 *          Handles command frame parsing and telemetry frame packing
 *          on top of the BSP CAN layer.
 *
 * @note    Protocol frame format (8 bytes):
 *          CMD frame:  [cmd_id:4B][cmd_data:4B]
 *          TELEM frame: [state:4B][velocity:2B][position:2B]
 ******************************************************************************
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "general_def.h"
#include "bsp_can.h"

/**
 * @brief CAN communication instance (wraps BSP CAN with protocol layer)
 */
typedef struct
{
    FDCANInstance *can_ins;     /* Underlying BSP CAN instance */
    uint8_t send_data_len;      /* TX payload length */
    uint8_t recv_data_len;      /* RX payload length */
} FDCANCommInstance;

/**
 * @brief CAN communication initialization config
 */
typedef struct
{
    FDCAN_Init_Config_s can_config;     /* BSP CAN config */
    uint8_t send_data_len;              /* TX data length */
    uint8_t recv_data_len;              /* RX data length */
} FDCANComm_Init_Config_s;

/* --- Public API --- */

/**
 * @brief  Initialize CAN communication instance
 * @param  config: pointer to init configuration
 * @return Pointer to CAN communication instance
 */
FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *config);

/**
 * @brief  Get received data from CAN buffer
 * @param  ins: CAN communication instance
 * @return Pointer to received data buffer
 */
void *FDCANCommGet(FDCANCommInstance *ins);

/**
 * @brief  Send data via CAN
 * @param  ins: CAN communication instance
 * @param  data: pointer to data to send
 * @return 0 on success
 */
int FDCANCommSend(FDCANCommInstance *ins, void *data);

#endif /* CAN_DRIVER_H */
