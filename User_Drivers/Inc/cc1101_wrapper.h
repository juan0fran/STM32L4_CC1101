#ifndef __CC1101_WRAPPER_H__
#define __CC1101_WRAPPER_H__

#include "stm32l4xx_hal.h"
#include "spi.h"
#include "utils.h"

#define MSLEEP(x) HAL_Delay(x)
#define MDELAY(x) MSLEEP(x)
#define SPI_TRANSFER(x, y, z)  spi_transfer(x, y, z)

#define CC11xx_GDO0()	HAL_GPIO_ReadPin(CC1101_GDO0_GPIO_Port, CC1101_GDO0_Pin)
#define CC11xx_GDO1()	HAL_GPIO_ReadPin(CC1101_GDO1_GPIO_Port, CC1101_GDO1_Pin)
#define CC11xx_GDO2()	HAL_GPIO_ReadPin(CC1101_GDO2_GPIO_Port, CC1101_GDO2_Pin)

#endif
