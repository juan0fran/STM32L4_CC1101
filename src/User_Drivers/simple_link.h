/***************************************************************************************************
*  File:        simple_link.h                                                                      *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: This file defines main simple_link structures and functions.                       *
*  Simple link is a UART protocol based on Kiss protocol                                           *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/


#ifndef __SIMPLE_LINK_H__
#define __SIMPLE_LINK_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos_util.h"
#include "utils.h"

#define SL_SIMPLE_LINK_MTU      1800
#define SL_HEADER_SIZE          6

#define SL_FRAME_END        0xC0
#define SL_FRAME_SCAPE      0xDB
#define SL_T_FRAME_END      0xDC
#define SL_T_FRAME_SCAPE    0xDD

/**
 * @brief Simple Link is a UART based link control
 *
 * This API aims to establish a common UART protocol
 * between different components, as two microcontrollers.
 * The protocol is based on asynchronous transmission
 * * It can recover from errors (link cut in the middle of a frame)
 * * Error detection through basic CRC-16
 * * Configurable length up to 65535 bytes (configured through MACRO SL_SIMPLE_LINK_MTU)
 * * 2 configuration bytes available for command passing or other stuff
 * * 2 synchronization bytes available for UART synchronization, to be set by both peers
 */

typedef enum simple_packet_state_e{
    sp_pos_config1,
    sp_pos_config2,
    sp_pos_len1,
    sp_pos_len2,
    sp_pos_crc1,
    sp_pos_crc2,
    sp_pos_payload,
    sp_pos_transpose_end,
    sp_pos_transpose_scape,
}simple_packet_state_e;

/**
 * @brief Structure explanation
 * Two structures are set for the link control, a structure that handles the packet
 * and a structure that handles the link control.
 * In case of the link control, you need to establish one structure for each transmission or reception
 * processes. Meaning that a single control structure can only handle TX or RX but not both (in general case)
 * Only for cases where TX and RX are sequential and the sync bytes are the same a structure can be used for both
 */
typedef union __attribute__ ((__packed__)) simple_link_packet_s{
    uint8_t     raw[SL_HEADER_SIZE + SL_SIMPLE_LINK_MTU];
    struct __attribute__ ((__packed__)) {
        uint8_t     config1;
        uint8_t     config2;
        uint16_t    len;
        uint16_t    crc;
        uint8_t     payload[SL_SIMPLE_LINK_MTU];
    }fields;
}simple_link_packet_t;

typedef struct __attribute__ ((__packed__)) simple_link_control_s{
    /* things here are used to control the link */
    /* PRIVATE ARGUMENTS */
    uint8_t     frame_scape_found;
    uint8_t     frame_end_found;
    uint32_t    byte_cnt;
}simple_link_control_t;

/**
 * @brief Starts the simple_link protocol or resets its values
 *
 * This function is the first very important to be executed, it sets the
 * synchronization bytes and an internal timeout for reception.
 *
 * @param[in] sync1 Synchronization byte number 1
 * @param[in] sync2 Synchronization byte number 2
 * @param[in] timeout Timeout in milliseconds between two character reception (prevent packet de-sync)
 * @returns 0 in case of OK, -1 in case of error
 */
int prepare_simple_link(simple_link_control_t * c);


/**
 * @brief Feeds the simple link receiver with a new byte
 *
 * @param[in] new_character Value of the new byte found over the UART link
 * @param[in] *c Control link structure pointer
 * @param[out] *p Packet structure pointer
 * @return -1 in case of assert in params, -2 as BAD crc, -3 as Bad length, 0 in case that
 *  new character correctly added to the packet and > 0 in case of packet received, where
 *  the value is the length of the whole packet structure (including headers)
 * In the packet structure, all the parameters are valid in this point
 *
 * This packet can be get by means of:
 *        while(1) {
 *            read(fd, &char, 1);
 *            if (get_simple_link_packet(c, ...) > 0) {
 *                // NEW PACKET ARRIVED!
 *            }
 *        }
 */
int get_simple_link_packet(uint8_t new_character, simple_link_control_t * c, simple_link_packet_t * p);

/**
 * @brief Creates a packet from a given buffer
 *
 * @param[in] *buffer Pointer to a buffer to be sent
 * @param[in] size Size of the buffer
 * @param[in] config1 Configuration byte number 1
 * @param[in] config2 Configuration byte number 2
 * @param[in] *c Pointer to link control structure
 * @param[out] *p Pointer to packet structure
 * @return -1 in case of error, size of the whole packet structure (full packet size including headers) if OK
 * In the packet structure, all the parameters are valid in this point
 *
 * Packet can be sent by means of: write(fd, &packet, size_returned);
 */
int set_simple_link_packet( void * buffer, size_t size,
                            uint8_t config1, uint8_t config2, simple_link_packet_t * p);

int send_kiss_packet(int fd, void * buffer, size_t size);

uint16_t _htons(uint16_t host);
uint16_t _ntohs(uint16_t network);

#endif
