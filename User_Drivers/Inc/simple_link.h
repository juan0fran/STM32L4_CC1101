#ifndef __USART_CONTROL_H__
#define __USART_CONTROL_H__


#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct serial_parms_s{
	int 			fd;
	int 			ret;
	unsigned char 	buffer[512];
	unsigned int 	timeout;
}serial_parms_t;

#define SIMPLE_LINK_MTU 1500
#define HEADER_SIZE 	8

typedef enum simple_packet_positions_e{
	sp_pos_sync1 = 0,
	sp_pos_sync2,
	sp_pos_config1,
	sp_pos_config2,
	sp_pos_len1,
	sp_pos_len2,
	sp_pos_crc1,
	sp_pos_crc2,
	sp_pos_payload,
}simple_packet_positions_e;

typedef union __attribute__ ((__packed__)) simple_link_packet_s{
	uint8_t 	raw[HEADER_SIZE + SIMPLE_LINK_MTU];
	struct __attribute__ ((__packed__)){
		uint8_t 	sync1;
		uint8_t 	sync2;
		uint8_t 	config1;
		uint8_t 	config2;
		uint16_t 	len;
		uint16_t 	crc;
		uint8_t 	payload[SIMPLE_LINK_MTU];
	}fields;
}simple_link_packet_t;

typedef struct __attribute__ ((__packed__)) simple_link_control_s{
	/* things here are used to control the link */
	/* PROTECTED ARGUMENTS -> to be set by prepare link function */
	uint8_t 	sync1;
	uint8_t 	sync2;
	/* sync1 and sync2 are the sync words to look for */
	/* PRIVATE ARGUMENTS */
	uint8_t 	sync1_found;
	uint8_t 	sync2_found;
	uint16_t 	byte_cnt;
	/* Public arguments, to be used by the user as OUTPUT */
	uint16_t    full_size;
	/* the rest is to manage the link, do not touch! */
}simple_link_control_t;

int prepare_simple_link(uint8_t sync1, uint8_t sync2, simple_link_control_t * c);

int get_simple_link_packet(uint8_t new_character, simple_link_control_t * h, simple_link_packet_t * p);

int set_simple_link_packet( uint8_t * buffer, uint16_t size, 
                            uint8_t config1, uint8_t config2,
                            simple_link_control_t * c, simple_link_packet_t * p);
#endif
