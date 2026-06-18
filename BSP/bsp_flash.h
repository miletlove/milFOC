/**
 ******************************************************************************
 * @file    bsp_flash.h
 * @author  Zhang jia ming (FalconFoc) / milFOC Team
 * @brief   Flash BSP — page-level erase/write API for STM32G431.
 *          Provides address macros for all 64 pages (2KB each).
 ******************************************************************************
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "general_def.h"
#include "stm32g431xx.h"

/* Page address macros (2KB per page, 64 pages = 128KB on STM32G431CB) */
#define ADDR_FLASH_PAGE_0  ((uint32_t)0x08000000)
#define ADDR_FLASH_PAGE_1  ((uint32_t)0x08000800)
#define ADDR_FLASH_PAGE_2  ((uint32_t)0x08001000)
#define ADDR_FLASH_PAGE_3  ((uint32_t)0x08001800)
#define ADDR_FLASH_PAGE_4  ((uint32_t)0x08002000)
#define ADDR_FLASH_PAGE_5  ((uint32_t)0x08002800)
#define ADDR_FLASH_PAGE_6  ((uint32_t)0x08003000)
#define ADDR_FLASH_PAGE_7  ((uint32_t)0x08003800)
#define ADDR_FLASH_PAGE_8  ((uint32_t)0x08004000)
#define ADDR_FLASH_PAGE_9  ((uint32_t)0x08004800)
#define ADDR_FLASH_PAGE_10 ((uint32_t)0x08005000)
#define ADDR_FLASH_PAGE_11 ((uint32_t)0x08005800)
#define ADDR_FLASH_PAGE_12 ((uint32_t)0x08006000)
#define ADDR_FLASH_PAGE_13 ((uint32_t)0x08006800)
#define ADDR_FLASH_PAGE_14 ((uint32_t)0x08007000)
#define ADDR_FLASH_PAGE_15 ((uint32_t)0x08007800)
#define ADDR_FLASH_PAGE_16 ((uint32_t)0x08008000)
#define ADDR_FLASH_PAGE_17 ((uint32_t)0x08008800)
#define ADDR_FLASH_PAGE_18 ((uint32_t)0x08009000)
#define ADDR_FLASH_PAGE_19 ((uint32_t)0x08009800)
#define ADDR_FLASH_PAGE_20 ((uint32_t)0x0800A000)
#define ADDR_FLASH_PAGE_21 ((uint32_t)0x0800A800)
#define ADDR_FLASH_PAGE_22 ((uint32_t)0x0800B000)
#define ADDR_FLASH_PAGE_23 ((uint32_t)0x0800B800)
#define ADDR_FLASH_PAGE_24 ((uint32_t)0x0800C000)
#define ADDR_FLASH_PAGE_25 ((uint32_t)0x0800C800)
#define ADDR_FLASH_PAGE_26 ((uint32_t)0x0800D000)
#define ADDR_FLASH_PAGE_27 ((uint32_t)0x0800D800)
#define ADDR_FLASH_PAGE_28 ((uint32_t)0x0800E000)
#define ADDR_FLASH_PAGE_29 ((uint32_t)0x0800E800)
#define ADDR_FLASH_PAGE_30 ((uint32_t)0x0800F000)
#define ADDR_FLASH_PAGE_31 ((uint32_t)0x0800F800)
#define ADDR_FLASH_PAGE_32 ((uint32_t)0x08010000)
#define ADDR_FLASH_PAGE_33 ((uint32_t)0x08010800)
#define ADDR_FLASH_PAGE_34 ((uint32_t)0x08011000)
#define ADDR_FLASH_PAGE_35 ((uint32_t)0x08011800)
#define ADDR_FLASH_PAGE_36 ((uint32_t)0x08012000)
#define ADDR_FLASH_PAGE_37 ((uint32_t)0x08012800)
#define ADDR_FLASH_PAGE_38 ((uint32_t)0x08013000)
#define ADDR_FLASH_PAGE_39 ((uint32_t)0x08013800)
#define ADDR_FLASH_PAGE_40 ((uint32_t)0x08014000)
#define ADDR_FLASH_PAGE_41 ((uint32_t)0x08014800)
#define ADDR_FLASH_PAGE_42 ((uint32_t)0x08015000)
#define ADDR_FLASH_PAGE_43 ((uint32_t)0x08015800)
#define ADDR_FLASH_PAGE_44 ((uint32_t)0x08016000)
#define ADDR_FLASH_PAGE_45 ((uint32_t)0x08016800)
#define ADDR_FLASH_PAGE_46 ((uint32_t)0x08017000)
#define ADDR_FLASH_PAGE_47 ((uint32_t)0x08017800)
#define ADDR_FLASH_PAGE_48 ((uint32_t)0x08018000)
#define ADDR_FLASH_PAGE_49 ((uint32_t)0x08018800)
#define ADDR_FLASH_PAGE_50 ((uint32_t)0x08019000)
#define ADDR_FLASH_PAGE_51 ((uint32_t)0x08019800)
#define ADDR_FLASH_PAGE_52 ((uint32_t)0x0801A000)
#define ADDR_FLASH_PAGE_53 ((uint32_t)0x0801A800)
#define ADDR_FLASH_PAGE_54 ((uint32_t)0x0801B000)
#define ADDR_FLASH_PAGE_55 ((uint32_t)0x0801B800)
#define ADDR_FLASH_PAGE_56 ((uint32_t)0x0801C000)
#define ADDR_FLASH_PAGE_57 ((uint32_t)0x0801C800)
#define ADDR_FLASH_PAGE_58 ((uint32_t)0x0801D000)
#define ADDR_FLASH_PAGE_59 ((uint32_t)0x0801D800)
#define ADDR_FLASH_PAGE_60 ((uint32_t)0x0801E000)
#define ADDR_FLASH_PAGE_61 ((uint32_t)0x0801E800)
#define ADDR_FLASH_PAGE_62 ((uint32_t)0x0801F000)
#define ADDR_FLASH_PAGE_63 ((uint32_t)0x0801F800)

uint8_t flash_bsp_erase_pages(uint32_t start_addr, uint32_t end_addr);
uint8_t flash_bsp_erase_page(uint8_t page);
void    flash_bsp_write_data(uint32_t addr, void *data, uint32_t size);

/* --- milFOC compatibility aliases (NVM abstraction) --- */
int  flash_erase_nvm(void);
int  flash_write_nvm(uint32_t addr, uint64_t *data, uint32_t len);
void flash_read_nvm(uint32_t addr, uint8_t *data, uint32_t len);

#endif /* BSP_FLASH_H */
