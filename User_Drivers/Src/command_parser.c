#include "command_parser.h"

extern circ_buff_t uart_queue;

static inline int uart_available(void){
	return available_items(&uart_queue);
}

static inline uint8_t read_uart(void){
	uint8_t val = 0x00;
	dequeue(&uart_queue, &val);
	return val;
}

void clear_uart(void){
	while(uart_available() > 0){
		read_uart();
	}
}

void remove_endlines(char * buff){
	int i, j;
	for( i = 0, j = 0; i < strlen(buff); i++){
		buff[i-j] = buff[i];
		if(buff[i] == '\n' || buff[i] == '\r'){
			j++;
		}
	}
	if(buff[i-j] == '\r' || buff[i-j] == '\n'){
		buff[i-j] = '\0';
	}else{
		buff[i] = '\0';
	}
}

int command_input(uint8_t * buffer, uint16_t max_count, uint32_t us_timeout){
	uint16_t cnt = 0;
	if (buffer == NULL || max_count == 0){
		return -1;
	}
	while (uart_available() > 0){
		buffer[cnt++] = read_uart();
		if (cnt == max_count){
			return max_count;
		}
		if (uart_available() == 0){
			delay_us(us_timeout);
		}
	}
	return cnt;
}

int command_input_until(uint8_t * char_set, uint8_t char_set_size, uint8_t * buffer, uint16_t max_count, uint32_t ms_timeout){
	uint16_t cnt = 0;
	uint8_t i;
	if (buffer == NULL || max_count == 0){
		return -1;
	}
	if (char_set == NULL || char_set_size == 0){
		return -1;
	}
	if (uart_available() > 0){
		memset(buffer, 0, max_count);
	}else{
		return -1;
	}
	do{
		buffer[cnt++] = read_uart();
		print_char(buffer[cnt-1]);
		for (i = 0; i < char_set_size; i++){
			if (buffer[cnt-1] == char_set[i]){
				return cnt;
			}
		}
		if (cnt == max_count){
			return max_count;
		}
		/* If nothing on UART && timeout not expired */
		/* Wait */
		while (uart_available() == 0 && ms_timeout-- > 0){
			delay_us(1000);
		}
		/* If this is passed, check the condition again and go! */
	}while (uart_available() > 0 && ms_timeout > 0);
	return cnt;
}


void init_command_handler(){
	usart_init_rx();
}
