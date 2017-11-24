#include "eeprom.h"

void write_eeprom(ep_eeprom_t data)
{
    /* write a eeprom structure */
    FLASH_EraseInitTypeDef erase_cmd;
    uint32_t page;
    int i;
    HAL_FLASH_Unlock();
    erase_cmd.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_cmd.Banks = FLASH_BANK_1;
    erase_cmd.Page = EP_FLASH_PAGE_START;
    erase_cmd.NbPages = 1;
    HAL_FLASHEx_Erase(&erase_cmd, &page);
    /* flash write */
    for(i = 0; i < EP_EEPROM_MAX_AMOUNT; i++) {
        HAL_FLASH_Program(    FLASH_TYPEPROGRAM_DOUBLEWORD,
                            (uint32_t) EP_EEPROM_START_ADDR+i*EP_EEPROM_ITEM_SIZE,
                            (uint64_t) data.raw[i]);
    }
    HAL_FLASH_Lock();
}

void read_eeprom(ep_eeprom_t *data)
{
    int i;
    const volatile uint64_t *aux;
    for(i = 0; i < EP_EEPROM_MAX_AMOUNT; i++) {
        aux = (const volatile uint64_t *) (EP_EEPROM_START_ADDR+i*EP_EEPROM_ITEM_SIZE);
        data->raw[i] = *aux;
    }
}
