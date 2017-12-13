/***************************************************************************************************
*  File:        housekeeping.h                                                                     *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: COMMS subsystems housekeeping definition                                           *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

#ifndef __HOUSEKEEPING_H__
#define __HOUSEKEEPING_H__

#include "freertos_util.h"

#define HK_TEMP_SENSOR_POS    0
#define HK_VBAT_SENSOR_POS    1
#define HK_VREF_SENSOR_POS    2

/* Change depending on the STM32 version */
#define HK_TEMP_CAL_REG_1     0x1FFF75A8
#define HK_TEMP_CAL_REG_2     0x1FFF75CA
#define HK_VREF_CAL_REG       0x1FFF75AA

#define HK_FLOAT_VOLTAGE      3.0
#define HK_TEMP_MEAS_1        30.0
#define HK_TEMP_MEAS_2        110.0
#define HK_TEMP_MEAS_DIFF     (HK_TEMP_MEAS_2 - HK_TEMP_MEAS_1)

#define HK_ADC_FULL_SCALE     4095

#define HK_BUFFER_SIZE        3

typedef struct
{
    uint16_t VREF;
    uint16_t TS_CAL_1;
    uint16_t TS_CAL_2;
}TSCALIB_t;

typedef struct __attribute__ ((__packed__)) comms_hk_data_u {
    struct __attribute__ ((__packed__)) {
        uint16_t    ext_temp;
        uint16_t    int_temp;
        uint32_t    phy_rx_packets;
        uint32_t    phy_tx_packets;
        uint16_t    phy_rx_errors;
        uint16_t    phy_tx_failed_packets;
        uint32_t    ll_rx_packets;
        uint32_t    ll_tx_packets;
        uint8_t     transmit_power;
        uint8_t     actual_rssi;
        uint8_t     last_rssi;
        uint8_t     last_lqi;
    }housekeeping;
    struct __attribute__ ((__packed__)) {
        uint8_t     rx_queued;
        uint8_t     tx_remaining;
        uint16_t    free_stack[4];
        uint16_t    used_stack[4];
    }control;
}comms_hk_data_t;

void         init_housekeeping(void);
void         refresh_housekeeping(void);
float        get_ref_voltage(void);
float        get_internal_temperature(void);
float        get_external_temperature(void);
float        get_voltage(void);

void ReturnHKData(comms_hk_data_t *data);

#endif /* INC_HOUSEKEEPING_H_ */
