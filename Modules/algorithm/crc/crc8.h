/**
 ******************************************************************************
 * @file    crc8.h
 * @author  milFOC Team (from FalconFoc)
 * @brief   CRC-8 calculation for data integrity verification.
 *          Supports both single-shot and streaming (update) modes.
 ******************************************************************************
 */

#ifndef CRC8_H
#define CRC8_H

#include "general_def.h"

#define CRC_START_8 0x00

uint8_t crc8_calc(const uint8_t *data, uint32_t len);
uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint8_t update_crc_8(uint8_t crc, uint8_t val);

#endif /* CRC8_H */
