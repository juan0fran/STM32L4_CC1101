#ifndef __CIRCULAR_QUEUE_H__
#define __CIRCULAR_QUEUE_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CIRC_BUFF_SIZE	2048

typedef struct circ_buff_s{
	uint8_t data[CIRC_BUFF_SIZE];
	uint16_t read_ptr;
	uint16_t write_ptr;
}circ_buff_t;

void queue_init(circ_buff_t * handler);
bool is_full(circ_buff_t * handler);
bool is_empty(circ_buff_t * handler);
bool dequeue(circ_buff_t * handler, uint8_t * val);
bool enqueue(circ_buff_t * handler, uint8_t val);


#endif
