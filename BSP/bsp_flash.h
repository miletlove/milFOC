/**
 ******************************************************************************
 * @file    bsp_flash.h
 * @author  milFOC Team
 * @brief   Internal Flash Board Support Package for STM32G431.
 *          Used for storing calibration parameters (encoder offset, motor R/L,
 *          PID gains, etc.) persistently.
 *
 * @note    STM32G431CBT6: 128 KB Flash, last page(s) reserved for NVM storage.
 *          Flash page size: 2 KB (for G4 series).
 ******************************************************************************
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "general_def.h"

/* Flash storage parameters */
#define FLASH_NVM_START_ADDR   ((uint32_t)0x0801F000)  /* Last 4KB reserved */
#define FLASH_NVM_PAGE_SIZE    2048                     /* G4 series: 2KB per page */
#define FLASH_NVM_SIZE         4096                     /* Reserved NVM area size */

/**
 * @brief  Erase the NVM flash page(s)
 * @return 0 on success, non-zero on failure
 */
int flash_erase_nvm(void);

/**
 * @brief  Write data to NVM flash area (must erase first)
 * @param  addr: target address within NVM area
 * @param  data: pointer to source data
 * @param  len: number of double-words (64-bit) to write
 * @return 0 on success, non-zero on failure
 */
int flash_write_nvm(uint32_t addr, uint64_t *data, uint32_t len);

/**
 * @brief  Read data from NVM flash area
 * @param  addr: source address within NVM area
 * @param  data: pointer to destination buffer
 * @param  len: number of bytes to read
 */
void flash_read_nvm(uint32_t addr, uint8_t *data, uint32_t len);

#endif /* BSP_FLASH_H */
