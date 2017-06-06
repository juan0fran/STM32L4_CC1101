#ifndef __CIRCULAR_QUEUE_H__
#define __CIRCULAR_QUEUE_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "freertos_util.h"

#define CIRC_BUFF_TOTAL_SIZE	(24 * 1024)

typedef struct mutex_handler_s{
	osMutexId 			m;
	osStaticMutexDef_t 	control;
}mutex_handler_t;

typedef struct circ_buff_s{
	void *  			data;
	uint16_t 			element_size;
	uint16_t 			element_count;
	volatile uint16_t	queued_items;
	volatile uint16_t 	read_ptr;
	volatile uint16_t 	write_ptr;
	uint32_t 			queue_size;
	mutex_handler_t		mutex;
}circ_buff_t;

void queue_init(circ_buff_t * handler, uint16_t element_size, uint16_t element_count);
bool dequeue(circ_buff_t * handler, void * val);
bool enqueue(circ_buff_t * handler, void * val);
uint16_t available_items(circ_buff_t * handler);
uint16_t available_space(circ_buff_t * handler);

#endif
