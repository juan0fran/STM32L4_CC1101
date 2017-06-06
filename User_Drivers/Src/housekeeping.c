/*
 * housekeeping.c
 *
 *  Created on: 17 de may. de 2017
 *      Author: cubecat
 */

#include "stm32l4xx_hal.h"
#include "housekeeping.h"
#include "cmsis_os.h"

static TSCALIB_t calib_data;
static uint16_t adc_buffer[HK_BUFFER_SIZE];
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

static void get_tscalib(TSCALIB_t *data)
{
	const volatile uint16_t *cal_temp_1;
	const volatile uint16_t *cal_temp_2;
	const volatile uint16_t *cal_vref;
	cal_temp_1 = (const volatile uint16_t *) HK_TEMP_CAL_REG_1;
	cal_temp_2 = (const volatile uint16_t *) HK_TEMP_CAL_REG_2;
	cal_vref = (const volatile uint16_t *) HK_VREF_CAL_REG;
	data->TS_CAL_1	= *cal_temp_1;
	data->TS_CAL_2 	= *cal_temp_2;
	data->VREF 		= *cal_vref;
}

void init_housekeeping()
{
	taskENTER_CRITICAL();
	get_tscalib(&calib_data);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	taskEXIT_CRITICAL();
	/* Now it starts doing shit */
}

static uint16_t get_ref_voltage()
{
	return (HK_VREF_VOLT_REF*calib_data.VREF/adc_buffer[HK_VREF_SENSOR_POS]);
}

void refresh_housekeeping()
{
	taskENTER_CRITICAL();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, HK_BUFFER_SIZE);
	taskEXIT_CRITICAL();
	while(hadc1.DMA_Handle->State != HAL_DMA_STATE_READY) {
		osDelay(1);
	}
}

int32_t get_external_temperature()
{
	uint8_t i2c_buffer[2];
	i2c_buffer[0] = 0x00;
	taskENTER_CRITICAL();
	HAL_I2C_Master_Transmit(&hi2c2, 0x4F<<1, i2c_buffer, 1, 1);
	HAL_I2C_Master_Receive(&hi2c2, 0x4F<<1, i2c_buffer, 2, 1);
	taskEXIT_CRITICAL();
	return i2c_buffer[0];
}

int32_t get_internal_temperature()
{
	/* correct for voltage */
	int32_t temp_cal;
	int32_t temp_uncal;
	temp_uncal = adc_buffer[HK_TEMP_SENSOR_POS] * get_ref_voltage()/HK_VREF_VOLT_REF;
	temp_cal = ( (int32_t) temp_uncal - (int32_t) calib_data.TS_CAL_1 );
	temp_cal *= (int32_t) HK_TEMP_MEAS_DIFF;
	temp_cal /= (int32_t) (calib_data.TS_CAL_2 - calib_data.TS_CAL_1);
	temp_cal += HK_TEMP_MEAS_1;
	return temp_cal;
}

uint32_t get_voltage()
{
	uint16_t volt_uncal = adc_buffer[HK_TEMP_SENSOR_POS] * get_ref_voltage()/HK_VREF_VOLT_REF;
	return (volt_uncal*3);
}
