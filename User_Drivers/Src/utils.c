/*
 * utils.c
 *
 *  Created on: Mar 17, 2017
 *      Author: gs-ms
 */

#include "utils.h"

int __errno;

void uart_send(void * p, uint16_t size)
{
	HAL_UART_Transmit(&huart1, p, size, 1*size);
}

int _write(int fd, void * p, size_t len)
{
	uart_send(p, len);
	return len;
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
