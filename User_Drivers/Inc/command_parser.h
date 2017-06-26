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

#include "link_layer.h"

typedef struct _cp_config_s {
	uint16_t reset_timeout;
}cp_config_t;

typedef enum _cp_command_type_e {
	data_frame,
	configuration_frame,
	request_frame,
	nack_frame,
}cp_command_type_e;

typedef union __attribute__ ((__packed__)) _cp_command_def_u {
	uint8_t raw[4];
	struct __attribute__ ((__packed__)){
		float freq_hz;
	}fields;
}cp_command_def_u;

void usart_rx_work(void);
void usart_tx_work(void);

#endif /* INC_COMMAND_PARSER_H_ */

