/*
 * utils.c
 *
 *  Created on: Mar 17, 2017
 *      Author: gs-ms
 */

#include "utils.h"
#include "freertos_util.h"

int __errno;

void _safe_send(void * p, uint16_t size)
{
	taskENTER_CRITICAL();
	HAL_UART_Transmit_IT(&huart1, p, size);
	taskEXIT_CRITICAL();
}

int _write(int fd, void *p, size_t len)
{
	/* must enqueue that and another1 process it */
	/* put this shit into a queue! */
	//uart_send(p, len);
	uint8_t *data = p;
	int i = 0;
	while(i < len) {
		osMessagePut(UartTxQueueHandle, data[i], osWaitForever);
		i++;
	}
	/* wait for that to end bro */
	return len;
}

void uart_send(void * p, uint16_t size)
{
	_write(0, p, size);
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
