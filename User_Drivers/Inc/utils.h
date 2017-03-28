/*
 * utils.h
 *
 *  Created on: Mar 17, 2017
 *      Author: gs-ms
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdarg.h>

#include "stm32l4xx_hal.h"
#include "tim.h"
#include "usart.h"

#define 	S_TO_US(x) 		x * 1000 * 1000
#define 	MS_TO_US(x)		x * 1000
#define 	US_TO_US(x)		x

#define 	S_TO_MS(x) 		x * 1000
#define 	MS_TO_MS(x)		x

void 		print_char(char character);
void 		print_uart_ln(char * fmt, ...);
void 		print_uart(char * fmt, ...);
void 		delay_us(uint32_t timeout);


#endif /* INC_UTILS_H_ */
