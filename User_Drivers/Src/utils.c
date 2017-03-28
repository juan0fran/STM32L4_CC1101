/*
 * utils.c
 *
 *  Created on: Mar 17, 2017
 *      Author: gs-ms
 */

#include "utils.h"


#define MILLISECOND_FROM_MICROSECOND		1000

void delay_us(uint32_t timeout){
	/* This is 1 millisecond */
	/* If timeout is greater than 1 millisecond... */
	/* Set 1 ms resolution */
	/* Otherwise, us resolution */
	/* By means of counting in 1ms we get that! */
	uint32_t millisecond_count;
	uint16_t microsecond_rem;
	if (timeout >= MILLISECOND_FROM_MICROSECOND){
		/* This is very probable... */
		/* Split in 1ms set */
		millisecond_count = timeout / 1000;
		microsecond_rem = timeout % 1000;
		while (millisecond_count--){
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_Base_Start(&htim2);
			while(__HAL_TIM_GET_COUNTER(&htim2) < 1000);
			HAL_TIM_Base_Stop(&htim2);
		}
		/* Now count microseconds */
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GET_COUNTER(&htim2) < microsecond_rem);
		HAL_TIM_Base_Stop(&htim2);
	}else{
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GET_COUNTER(&htim2) < timeout);
		HAL_TIM_Base_Stop(&htim2);
	}
}

void print_char(char character)
{
	volatile static char last_char = 0;
	uint8_t send_buff[3];
	if (last_char != '\r' && character == '\n'){
		character = '\r';
		send_buff[0] = '\r';
		send_buff[1] = '\n';
		send_buff[2] = '\0';
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *) send_buff, 3);
		while(huart1.hdmatx->State != HAL_DMA_STATE_READY /* or timeout */);
	}else{
		if (character == '\r'){
			send_buff[0] = '\r';
			send_buff[1] = '\n';
			send_buff[2] = '\0';
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *) send_buff, 3);
			while(huart1.hdmatx->State != HAL_DMA_STATE_READY /* or timeout */);
		}else{
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *) &character, 1);
			while(huart1.hdmatx->State != HAL_DMA_STATE_READY /* or timeout */);
		}
	}
}

void print_uart(char * fmt, ...)
{
	/* use of vsprintf */
	char print_buffer[256];
	memset(print_buffer, 0 , sizeof(print_buffer));
	va_list args;
	va_start (args, fmt);
	vsprintf (print_buffer, fmt, args);
	va_end (args);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) print_buffer, strlen((const char *) print_buffer));
	while(huart1.hdmatx->State != HAL_DMA_STATE_READY /* or timeout */);
}

void print_uart_ln(char * fmt, ...)
{
	char print_buffer[256];
	memset(print_buffer, 0 , sizeof(print_buffer));
	va_list args;
	va_start (args, fmt);
	vsprintf (print_buffer, fmt, args);
	va_end (args);
	strcat(print_buffer, "\r\n");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) print_buffer, strlen((const char *) print_buffer));
	while(huart1.hdmatx->State != HAL_DMA_STATE_READY /* or timeout */);
}
