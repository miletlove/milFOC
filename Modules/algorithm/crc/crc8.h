/**
 ******************************************************************************
 * @file    crc8.h
 * @author  milFOC Team
 * @brief   CRC-8 calculation for data integrity verification.
 ******************************************************************************
 */

#ifndef CRC8_H
#define CRC8_H

#include "general_def.h"

uint8_t crc8_calc(const uint8_t *data, uint32_t len);

#endif /* CRC8_H */
