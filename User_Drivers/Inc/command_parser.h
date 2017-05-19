/*
 * command_parser.h
 *
 *  Created on: Mar 16, 2017
 *      Author: gs-ms
 */

#ifndef INC_COMMAND_PARSER_H_
#define INC_COMMAND_PARSER_H_

#include "utils.h"
#include "usart.h"
#include "simple_link.h"
#include "circular_queue.h"
#include "link_layer.h"

extern circ_buff_t uart_queue;

typedef enum _cp_command_type_e {
	data_frame,
	configuration,
}cp_command_type_e;

typedef union __attribute__ ((__packed__)) _cp_command_def_u {
	uint8_t raw[4];
	struct __attribute__ ((__packed__)){
		float freq_hz;
	}fields;
}cp_command_def_u;

void	init_command_handler();
int 	retrieve_command(simple_link_packet_t *s_packet);

#endif /* INC_COMMAND_PARSER_H_ */

