/*
 * housekeeping.h
 *
 *  Created on: 17 de may. de 2017
 *      Author: cubecat
 */

#ifndef INC_HOUSEKEEPING_H_
#define INC_HOUSEKEEPING_H_

#define HK_TEMP_SENSOR_POS	0
#define HK_VBAT_SENSOR_POS	1
#define HK_VREF_SENSOR_POS	2

/* Change depending on the STM32 version */
#define HK_TEMP_CAL_REG_1	0x1FFF75A8
#define HK_TEMP_CAL_REG_2	0x1FFF75CA
#define HK_VREF_CAL_REG		0x1FFF75AA

#define HK_VREF_VOLT_REF	3000
#define HK_TEMP_MEAS_1		30
#define HK_TEMP_MEAS_2		110
#define HK_TEMP_MEAS_DIFF	(HK_TEMP_MEAS_2 - HK_TEMP_MEAS_1)

#define HK_BUFFER_SIZE		3

typedef struct
{
	uint16_t VREF;
	uint16_t TS_CAL_1;
	uint16_t TS_CAL_2;
}TSCALIB_t;

typedef struct __attribute__ ((__packed__)) comms_hk_data_u {
	struct __attribute__ ((__packed__)) {
		uint16_t 	ext_temp;
		uint16_t 	int_temp;
		uint32_t	phy_rx_packets;
		uint32_t	phy_tx_packets;
		uint16_t 	phy_rx_errors;
		uint16_t	phy_tx_failed_packets;
		uint32_t 	ll_rx_packets;
		uint32_t 	ll_tx_packets;
		uint8_t		transmit_power;
		uint8_t		actual_rssi;
		uint8_t 	last_rssi;
		uint8_t 	last_lqi;
	}housekeeping;
	struct __attribute__ ((__packed__)) {
		uint8_t 	rx_queued;
		uint8_t		tx_remaining;
		uint16_t	free_stack[4];
		uint16_t	used_stack[4];
	}control;
}comms_hk_data_t;

void 		init_housekeeping();
void 		refresh_housekeeping();
float 		get_internal_temperature();
float 		get_external_temperature();
float	 	get_voltage();

#endif /* INC_HOUSEKEEPING_H_ */

