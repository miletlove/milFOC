/**
 ******************************************************************************
 * @file    flash_driver.h
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   Flash driver module — high-level page erase / read / write using
 *          the BSP flash API (bsp_flash.h).
 ******************************************************************************
 */

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include "general_def.h"
#include "bsp_flash.h"

uint8_t flash_erase_page(uint32_t page);
uint8_t flash_erase_pages(uint32_t start_add, uint32_t end_add);
void    flash_write_data(uint32_t addr, void *data, uint32_t size);
void    flash_read_data(uint32_t addr, uint32_t *data, uint32_t size);

#endif /* FLASH_DRIVER_H */
