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
	volatile uint32_t millisecond_count;
	volatile uint16_t microsecond_rem;
	if (timeout >= MILLISECOND_FROM_MICROSECOND){
		/* This is very probable... */
		/* Split in 1ms set */
		millisecond_count = timeout / MILLISECOND_FROM_MICROSECOND;
		microsecond_rem = timeout % MILLISECOND_FROM_MICROSECOND;
		while (millisecond_count--){
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_Base_Start(&htim2);
			while(__HAL_TIM_GET_COUNTER(&htim2) < MILLISECOND_FROM_MICROSECOND){asm("NOP");};
			HAL_TIM_Base_Stop(&htim2);
		}
		/* Now count microseconds */
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GET_COUNTER(&htim2) < microsecond_rem){asm("NOP");};
		HAL_TIM_Base_Stop(&htim2);
	}else{
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start(&htim2);
		while(__HAL_TIM_GET_COUNTER(&htim2) < timeout){asm("NOP");};
		HAL_TIM_Base_Stop(&htim2);
	}
}

void uart_send(void * p, uint16_t size)
{
	HAL_UART_Transmit(&huart1, p, size, 1*size);
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
		uart_send(send_buff, 3);
	}else{
		if (character == '\r'){
			send_buff[0] = '\r';
			send_buff[1] = '\n';
			send_buff[2] = '\0';
			uart_send(send_buff, 3);
		}else{
			send_buff[0] = character;
			uart_send(send_buff, 1);
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
	uart_send((uint8_t *) print_buffer, strlen((const char *) print_buffer));
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
	uart_send((uint8_t *) print_buffer, strlen((const char *) print_buffer));
}

