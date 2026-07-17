/**
 * ============================================================================
 *  内部 Flash 模拟 EEPROM 存储模块实现
 *  利用 HAL Flash API 擦写 Page63
 * ============================================================================
 */
#include "bsp_flash_storage.h"
#include <string.h>

/* ======================== 公开接口 ======================== */

bool Flash_Load(FlashStorage_t *data)
{
    /* 直接从 Flash 地址读取 (Flash 在地址空间中可直接指针访问) */
    memcpy(data, (const void *)FLASH_STORAGE_PAGE_ADDR, sizeof(FlashStorage_t));

    /* 魔数校验 */
    if (data->magic != FLASH_STORAGE_MAGIC) {
        return false;
    }
    return true;
}

bool Flash_Save(const FlashStorage_t *data)
{
    HAL_StatusTypeDef ret;
    FLASH_EraseInitTypeDef erase_cfg;
    uint32_t page_err = 0;
    uint32_t addr = FLASH_STORAGE_PAGE_ADDR;
    const uint32_t *src = (const uint32_t *)data;
    uint32_t words = (sizeof(FlashStorage_t) + 3) / 4;  /* 向上取整到 4 字节 */

    /* 解锁 Flash */
    ret = HAL_FLASH_Unlock();
    if (ret != HAL_OK) return false;

    /* 擦除 Page63 */
    erase_cfg.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_cfg.PageAddress = FLASH_STORAGE_PAGE_ADDR;
    erase_cfg.NbPages     = 1;
    ret = HAL_FLASHEx_Erase(&erase_cfg, &page_err);
    if (ret != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    /* 按 32 位字写入 */
    for (uint32_t i = 0; i < words; i++) {
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
        if (ret != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return true;
}
