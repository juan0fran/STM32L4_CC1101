/***************************************************************************************************
*  File:        command_parser.h                                                                   *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: Command parser defintion, includes command definitions of UART interface           *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

#ifndef __COMMAND_PARSER_H__
#define __COMMAND_PARSER_H__

#include "utils.h"
#include "simple_link.h"
#include "freertos_util.h"
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

void usart_work(void);

#endif /* INC_COMMAND_PARSER_H_ */
