#include "circular_queue.h"

static struct memory_s{
	uint8_t 	available_space [CIRC_BUFF_TOTAL_SIZE];
	uint32_t 	mem_ptr;
}memory_t;

static void* allocate_memory(uint32_t size){

	if ((size+memory_t.mem_ptr) > CIRC_BUFF_TOTAL_SIZE){
		return NULL;
	}else{
		memory_t.mem_ptr += size;
		return &memory_t.available_space[memory_t.mem_ptr];
	}
	return NULL;
}

void queue_init(circ_buff_t * handler, uint16_t element_size, uint16_t element_count)
{
	if (handler->data == NULL){
		handler->data = allocate_memory(element_size * element_count);
		if (handler->data == NULL){
			return;
		}
		//memset(handler->data, 0, handler->queue_size);
		handler->read_ptr = 0;
		handler->write_ptr = 0;
		handler->element_size = element_size;
		handler->queue_size = element_size * element_count;
	}
}

bool is_full(circ_buff_t * handler)
{
	return (((handler->write_ptr + handler->element_size) % handler->queue_size) == handler->read_ptr) ? true : false;
}

bool is_empty(circ_buff_t * handler)
{
	return (handler->write_ptr == handler->read_ptr) ? true : false;
}

bool enqueue(circ_buff_t * handler, void * val)
{
	if (handler->element_size == 0){
		return false;
	}
	if (!is_full(handler)){
		memcpy(handler->data+handler->write_ptr, val, handler->element_size);
		//handler->data[handler->write_ptr] = val;
		handler->write_ptr = handler->write_ptr + handler->element_size;
		handler->write_ptr %= handler->queue_size;
		return true;
	}else{
		return false;
	}
}

bool dequeue(circ_buff_t * handler, void * val)
{
	if (handler->element_size == 0){
		return false;
	}
	if (!is_empty(handler)){
		memcpy(val, handler->data+handler->read_ptr, handler->element_size);
		//*val = handler->data[handler->read_ptr];
		handler->read_ptr = handler->read_ptr + handler->element_size;
		handler->read_ptr %= handler->queue_size;
		return true;
	}else{
		return false;
	}
}
