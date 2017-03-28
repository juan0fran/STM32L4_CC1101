/*
 * command_parser.h
 *
 *  Created on: Mar 16, 2017
 *      Author: gs-ms
 */

#ifndef INC_COMMAND_PARSER_H_
#define INC_COMMAND_PARSER_H_

#include <circular_queue.h>
#include "utils.h"
#include "usart.h"

void 		clear_uart(void);
void 		remove_endlines(char * buff);
int 		command_input_until(uint8_t * char_set, uint8_t char_set_size, uint8_t * buffer, uint16_t max_count, uint32_t us_timeout);
void 		init_command_handler();

#endif /* INC_COMMAND_PARSER_H_ */
