/**
 ******************************************************************************
 * @file    crc16.h
 * @author  milFOC Team
 * @brief   CRC-16 calculation for data integrity verification.
 ******************************************************************************
 */

#ifndef CRC16_H
#define CRC16_H

#include "general_def.h"

uint16_t crc16_calc(const uint8_t *data, uint32_t len);

#endif /* CRC16_H */
