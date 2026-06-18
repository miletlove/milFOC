/**
 ******************************************************************************
 * @file    flash_driver.c
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   Flash driver implementation — thin wrapper over bsp_flash API.
 ******************************************************************************
 */

#include "flash_driver.h"
#include "string.h"

uint8_t flash_erase_page(uint32_t page)
{
    return flash_bsp_erase_page((uint8_t)page);
}

uint8_t flash_erase_pages(uint32_t start_add, uint32_t end_add)
{
    return flash_bsp_erase_pages(start_add, end_add);
}

void flash_write_data(uint32_t addr, void *data, uint32_t size)
{
    flash_bsp_write_data(addr, data, size);
}

void flash_read_data(uint32_t addr, uint32_t *data, uint32_t size)
{
    memcpy(data, (uint32_t *)addr, size);
}
