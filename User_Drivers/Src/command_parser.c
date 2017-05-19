#include "command_parser.h"

static simple_link_control_t s_control;


void init_command_handler()
{
	usart_init_rx(8192);
	prepare_simple_link(&s_control);
}

/* Check for commands */
int retrieve_command(simple_link_packet_t *s_packet)
{
	uint8_t byte;
	if (available_items(&uart_queue) > 0) {
		 while (dequeue(&uart_queue, &byte)) {
			 if( get_simple_link_packet(byte, &s_control, s_packet) > 0) {
				 return 1;
			 }
		 }
	}else {
		return 0;
	}
	return 0;
}

int process_command(simple_link_packet_t *packet)
{
	if (packet->fields.config1 == (uint8_t) data_frame) {
		/* busy waiting till is finished or data storage? */

	}else {
		/* whatever */
	}
	return 0;
}
