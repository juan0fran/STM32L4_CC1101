/***************************************************************************************************
*  File:        utils.h                                                                            *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: This file includes all util common definitions and and funcions of the sat.        *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdarg.h>

#include <stdio.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include <stdlib.h>
#include <string.h>

#include "stm32l4xx_hal.h"

#define CC1101_CS_Pin GPIO_PIN_4
#define CC1101_CS_GPIO_Port GPIOA
#define CC1101_GDO0_Pin GPIO_PIN_4
#define CC1101_GDO0_GPIO_Port GPIOC
#define CC1101_GDO0_EXTI_IRQn EXTI4_IRQn
#define CC1101_GDO2_Pin GPIO_PIN_5
#define CC1101_GDO2_GPIO_Port GPIOC
#define CC1101_GDO2_EXTI_IRQn EXTI9_5_IRQn
#define SW_ENABLE_Pin GPIO_PIN_0
#define SW_ENABLE_GPIO_Port GPIOB
#define SW_CONTROL_Pin GPIO_PIN_1
#define SW_CONTROL_GPIO_Port GPIOB
#define PA_EN_Pin GPIO_PIN_12
#define PA_EN_GPIO_Port GPIOB
#define _LNA_EN_Pin GPIO_PIN_14
#define _LNA_EN_GPIO_Port GPIOB
#define _5V_RF_EN_Pin GPIO_PIN_15
#define _5V_RF_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define CC1101_CS_Pin               GPIO_PIN_4
#define CC1101_CS_GPIO_Port         GPIOA

#define CC1101_GDO2_Pin             GPIO_PIN_5
#define CC1101_GDO2_GPIO_Port       GPIOC

#define CC1101_GDO0_Pin             GPIO_PIN_4
#define CC1101_GDO0_GPIO_Port       GPIOC

#define CC1101_GDO2_EXTI_IRQn       EXTI9_5_IRQn
#define CC1101_GDO0_EXTI_IRQn       EXTI4_IRQn

#define CC1101_GDO1_GPIO_Port       GPIOA
#define CC1101_GDO1_Pin             GPIO_PIN_6

/* USER CODE END Private defines */

extern int __errno;

#define     S_TO_US(x)         x * 1000 * 1000
#define     MS_TO_US(x)        x * 1000
#define     US_TO_US(x)        x

#define     S_TO_MS(x)         x * 1000
#define     MS_TO_MS(x)        x

void        check_for_printf_buffer(void);

void        print_char(char character);
void        print_uart_ln(char * fmt, ...);
void        print_uart(char * fmt, ...);
void        delay_us(uint32_t timeout);

int         uart_send(void * p, uint16_t size);
void        _safe_send(void * p, uint16_t size);

float       convert_temp_u16_f(uint16_t temp);
uint16_t    convert_temp_f_u16(float temp);

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c2;

extern IWDG_HandleTypeDef hiwdg;

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

int spi_transfer(uint8_t * ptx, uint8_t * prx, uint8_t len);

void usart_init_tx(void);
void usart_init_rx(void);

void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif /* INC_UTILS_H_ */
