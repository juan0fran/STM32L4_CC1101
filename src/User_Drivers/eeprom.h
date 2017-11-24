/*
 * eeprom.h
 *
 *  Created on: 9 de may. de 2017
 *      Author: cubecat
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32l4xx_hal.h"

#define EP_FLASH_START_ADDR        0x08000000
#define EP_FLASH_PAGE_SIZE         0x800
#define EP_FLASH_PAGE_START        128

/* Start address of eeprom memory */
#define EP_EEPROM_START_ADDR    (EP_FLASH_START_ADDR + (EP_FLASH_PAGE_SIZE*EP_FLASH_PAGE_START))

#define EP_EEPROM_MAX_AMOUNT    12
#define EP_EEPROM_ITEM_SIZE        (sizeof(uint64_t))

typedef union __attribute__ ((__packed__)) ep_eeprom_s {
    uint64_t raw[EP_EEPROM_MAX_AMOUNT];
    struct __attribute__ ((__packed__)) {
        uint8_t array[EP_EEPROM_ITEM_SIZE * EP_EEPROM_MAX_AMOUNT];
    }fields;
}ep_eeprom_t;

void write_eeprom(ep_eeprom_t data);
void read_eeprom(ep_eeprom_t *data);

#endif /* INC_EEPROM_H_ */
