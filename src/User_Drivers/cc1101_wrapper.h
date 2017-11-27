/***************************************************************************************************
*  File:        cc1101_wrapper.h                                                                   *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: CC1101 include util file, includes wrappers for GPIO, SPI and Sleep functions      *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

#ifndef __CC1101_WRAPPER_H__
#define __CC1101_WRAPPER_H__

#include "stm32l4xx_hal.h"
#include "utils.h"
#include "freertos_util.h"

#define MSLEEP(x)               vTaskDelay(x)
#define MDELAY(x)               MSLEEP(x)
#define SPI_TRANSFER(x, y, z)   spi_transfer(x, y, z)

#define CC11xx_GDO0()    HAL_GPIO_ReadPin(CC1101_GDO0_GPIO_Port, CC1101_GDO0_Pin)
#define CC11xx_GDO1()    HAL_GPIO_ReadPin(CC1101_GDO1_GPIO_Port, CC1101_GDO1_Pin)
#define CC11xx_GDO2()    HAL_GPIO_ReadPin(CC1101_GDO2_GPIO_Port, CC1101_GDO2_Pin)

#endif
