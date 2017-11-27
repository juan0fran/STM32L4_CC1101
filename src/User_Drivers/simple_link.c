/***************************************************************************************************
*  File:        simple_link.c                                                                      *
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

#include "simple_link.h"

static uint16_t crc_table [256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
    0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
    0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
    0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
    0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969,
    0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
    0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6,
    0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1,
    0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
    0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447,
    0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
    0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
    0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0,
    0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
    0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

/* Check system endianess */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static uint16_t swap_uint16(uint16_t s)
{
    return ( ((s & 0xFF) << 8) + ((s >> 8) & 0xFF) );
}
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
static uint16_t swap_uint16(uint16_t s)
{
    return s;
}
#else
static uint16_t swap_uint16(uint16_t s)
{
    return ( ((s & 0xFF) << 8) + ((s >> 8) & 0xFF) );
}
#endif

uint16_t _htons(uint16_t host)
{
    return (swap_uint16(host));
}

uint16_t _ntohs(uint16_t network)
{
    return (swap_uint16(network));
}

static uint16_t crc16_ccitt(uint8_t *data, uint16_t length, uint16_t seed, uint16_t final)
{
   uint16_t count;
   uint16_t crc = seed;
   uint16_t temp;
   for(count = 0; count < length; ++count) {
     temp = ( (*data++ ^ (crc >> 8)) & 0xff );
     crc = crc_table[temp] ^ (crc << 8);
   }
   return (uint16_t)(crc ^ final);
}

int send_kiss_packet(int fd, void * p, size_t size)
{
    uint8_t *buffer;
    uint8_t aux[2];
    int i = 0;
    int ret = 0;
    if(xSemaphoreTake(SimpleLinkMutexHandle, 1000) == pdTRUE) {
        if(p != NULL) {
            buffer = (uint8_t *) p;
            aux[0] = SL_FRAME_END;
            if(uart_send(aux, 1) != 1) {
                ret = -1;
                goto _func_error;
            }
            while(i < size) {
                if(buffer[i] == SL_FRAME_END) {
                    aux[0] = SL_FRAME_SCAPE;
                    aux[1] = SL_T_FRAME_END;
                    if(uart_send(aux, 2) != 2) {
                        ret = -1;
                        goto _func_error;
                    }
                } else if(buffer[i] == SL_FRAME_SCAPE) {
                    aux[0] = SL_FRAME_SCAPE;
                    aux[1] = SL_T_FRAME_SCAPE;
                    if(uart_send(aux, 2) != 2) {
                        ret = -1;
                        goto _func_error;
                    }
                } else {
                    if(uart_send(&buffer[i], 1) != 1) {
                        ret = -1;
                        goto _func_error;
                    }
                }
                i++;
            }
            aux[0] = SL_FRAME_END;
            if(uart_send(aux, 1) != 1) {
                ret = -1;
                goto _func_error;
            }
        } else {
            ret = -1;
            goto _func_error;
        }
    }
    _func_error:
    xSemaphoreGive(SimpleLinkMutexHandle);
    xTaskNotify(InterfaceTaskHandle, IFACE_NOTIFY_TX_REQ, eSetBits);
    return ret;
}

/* You give a buffer, it headers it with some info */
/* It direclty copies the content of the buffer into the simple_link_packet handler */
int set_simple_link_packet( void *buffer, size_t size,
                            uint8_t config1, uint8_t config2, simple_link_packet_t *p)
{
    if( (buffer == NULL && size > 0) || size > SL_SIMPLE_LINK_MTU || p == NULL) {
        return -1;
    }

    if(buffer != NULL && size > 0) {
        if(memcmp(p->fields.payload, buffer, size) != 0) {
            memcpy(p->fields.payload, buffer, size);
        }
    }

    p->fields.config1 = config1;
    p->fields.config2 = config2;

    p->fields.len = _htons(size);
    /* Compute and append CRC */
    p->fields.crc = crc16_ccitt(p->fields.payload, size, 0xFFFF, 0);
    p->fields.crc = _htons(p->fields.crc);

    return (size + SL_HEADER_SIZE);
}

/* To be executed once */
/* Set syncwords here, set control bytes (for TX purposes) */
int prepare_simple_link(simple_link_control_t * c)
{
    if(c == NULL) {
        return -1;
    }
    c->frame_scape_found = 0;
    c->frame_end_found = 0;
    c->byte_cnt = 0;
    return 0;
}

/* Feed with input bytes */
/* It outputs the raw packet (with the things as they are, in network byte endianess) */
/* But it outputs the control struct, where the control bytes are appended, the frame length and more control opts */
int get_simple_link_packet(uint8_t new_character, simple_link_control_t *c, simple_link_packet_t *p)
{
    int ret = 0;
    if(c == NULL || p == NULL) {
        return -1;
    }
    if(c->frame_end_found == 0) {
        if(new_character == SL_FRAME_END) {
            memset(p, 0, sizeof(simple_link_packet_t));
            prepare_simple_link(c);
            c->frame_end_found = 1;
        }
    } else if(c->byte_cnt == 0 && new_character == SL_FRAME_END) {
        memset(p, 0, sizeof(simple_link_packet_t));
        prepare_simple_link(c);
        c->frame_end_found = 1;
    } else if(c->frame_end_found == 1) {
        if(new_character == SL_FRAME_END) {
            p->fields.len = _ntohs(p->fields.len);
            p->fields.crc = _ntohs(p->fields.crc);
            if(crc16_ccitt(p->fields.payload, p->fields.len, 0xFFFF, p->fields.crc) == 0) {
                ret = p->fields.len + SL_HEADER_SIZE;
            } else {
                ret = 0;
            }
            prepare_simple_link(c);
        } else if(c->frame_scape_found == 1) {
            if(new_character == SL_T_FRAME_END) {
                p->raw[c->byte_cnt] = SL_FRAME_END;
            } else if(new_character == SL_T_FRAME_SCAPE) {
                p->raw[c->byte_cnt] = SL_FRAME_SCAPE;
            }
            c->frame_scape_found = 0;
            c->byte_cnt++;
        } else if(new_character == SL_FRAME_SCAPE) {
            c->frame_scape_found = 1;
        } else {
            p->raw[c->byte_cnt] = new_character;
            c->byte_cnt++;
        }
    } else {
        c->frame_end_found = 0;
        c->byte_cnt = 0;
        ret = 0;
    }
    return ret;
}
