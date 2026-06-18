/**
 ******************************************************************************
 * @file    crc16.h
 * @author  milFOC Team (from FalconFoc)
 * @brief   CRC-16 calculation for data integrity verification.
 *          Supports CRC-16, Modbus CRC, and streaming (update) modes.
 ******************************************************************************
 */

#ifndef CRC16_H
#define CRC16_H

#include "general_def.h"

#define CRC_START_16     0xFFFF
#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16      0xA001

uint16_t crc16_calc(const uint8_t *data, uint32_t len);
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);
uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes);
uint16_t update_crc_16(uint16_t crc, uint8_t c);
void     init_crc16_tab(void);

#endif /* CRC16_H */
