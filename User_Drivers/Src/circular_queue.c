#include "circular_queue.h"

void queue_init(circ_buff_t * handler)
{
	memset(handler->data, 0, CIRC_BUFF_SIZE);
	handler->read_ptr = 0;
	handler->write_ptr = 0;
}

bool is_full(circ_buff_t * handler)
{
	return (((handler->write_ptr+1) % CIRC_BUFF_SIZE) == handler->read_ptr) ? true : false;
}

bool is_empty(circ_buff_t * handler)
{
	return (handler->write_ptr == handler->read_ptr) ? true : false;
}

bool enqueue(circ_buff_t * handler, uint8_t val)
{
	if (!is_full(handler)){
		handler->data[handler->write_ptr] = val;
		handler->write_ptr = handler->write_ptr+1;
		handler->write_ptr %= CIRC_BUFF_SIZE;
		return true;
	}else{
		return false;
	}
}

bool dequeue(circ_buff_t * handler, uint8_t * val)
{
	if (!is_empty(handler)){
		*val = handler->data[handler->read_ptr];
		handler->read_ptr = handler->read_ptr+1;
		handler->read_ptr %= CIRC_BUFF_SIZE;
		return true;
	}else{
		return false;
	}
}
