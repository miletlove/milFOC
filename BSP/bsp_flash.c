/**
 ******************************************************************************
 * @file    bsp_flash.c
 * @author  milFOC Team
 * @brief   Internal Flash BSP implementation for STM32G431 NVM storage.
 *
 * @note    Uses HAL_FLASHEx APIs for G4 series.
 *          Flash must be erased per page before programming.
 *          All operations require disabling IRQs during flash write.
 ******************************************************************************
 */

#include "bsp_flash.h"
#include "bsp_log.h"
#include "string.h"

/**
 * @brief  Erase NVM flash pages
 */
int flash_erase_nvm(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;

    /* Unlock flash */
    HAL_FLASH_Unlock();

    /* Configure erase */
    erase_init.TypeErase    = FLASH_TYPEERASE_PAGES;
    erase_init.Banks        = FLASH_BANK_1;
    erase_init.Page         = (FLASH_NVM_START_ADDR - 0x08000000) / FLASH_NVM_PAGE_SIZE;
    erase_init.NbPages      = FLASH_NVM_SIZE / FLASH_NVM_PAGE_SIZE;

    __disable_irq();
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    __enable_irq();

    HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        LOGERROR("[FLASH] Erase failed at page %lu", page_error);
        return -1;
    }

    LOGINFO("[FLASH] NVM pages erased (addr=0x%08X, size=%d)", FLASH_NVM_START_ADDR, FLASH_NVM_SIZE);
    return 0;
}

/**
 * @brief  Write 64-bit double-words to NVM flash
 */
int flash_write_nvm(uint32_t addr, uint64_t *data, uint32_t len)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (addr < FLASH_NVM_START_ADDR || (addr + len * 8) > (FLASH_NVM_START_ADDR + FLASH_NVM_SIZE))
    {
        LOGERROR("[FLASH] Write address out of NVM range");
        return -1;
    }

    HAL_FLASH_Unlock();

    __disable_irq();
    for (uint32_t i = 0; i < len; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i * 8, data[i]);
        if (status != HAL_OK) break;
    }
    __enable_irq();

    HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        LOGERROR("[FLASH] Write failed at addr=0x%08X", addr);
        return -1;
    }

    return 0;
}

/**
 * @brief  Read from NVM flash area
 */
void flash_read_nvm(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (addr < FLASH_NVM_START_ADDR || (addr + len) > (FLASH_NVM_START_ADDR + FLASH_NVM_SIZE))
    {
        LOGERROR("[FLASH] Read address out of NVM range");
        return;
    }
    memcpy(data, (void *)addr, len);
}
