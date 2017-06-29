/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     

#include "cc1101_routine.h"
#include "command_parser.h"
#include "housekeeping.h"

#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId ControlTaskHandle;
uint32_t ControlTaskBuffer[ 256 ];
osStaticThreadDef_t ControlTaskControlBlock;

osThreadId CommsTaskHandle;
uint32_t CommsBuffer[ 768 ];
osStaticThreadDef_t CommsControlBlock;

osThreadId CommsTxTaskHandle;
uint32_t CommsTxBuffer[ 256 ];
osStaticThreadDef_t CommsTxControlBlock;

osThreadId InterfaceTaskHandle;
uint32_t InterfaceBuffer[ 256 ];
osStaticThreadDef_t InterfaceControlBlock;

osMessageQId UartQueueTxHandle;
uint8_t UartQueueTxBuffer[ 4096 * sizeof( uint8_t ) ];
osStaticMessageQDef_t UartQueueTxControlBlock;

osMessageQId UartQueueRxHandle;
uint8_t UartQueueRxBuffer[ 4096 * sizeof( uint8_t ) ];
osStaticMessageQDef_t UartQueueRxControlBlock;

/* USER CODE BEGIN Variables */
simple_link_packet_t LinkLayerRxBuffer[ 6 ];
osStaticMessageQDef_t LinkLayerRxControlBlock;
osMessageQId LinkLayerRxQueueHandle;

simple_link_packet_t RadioPacketTxBuffer[ 2 ];
osStaticMessageQDef_t RadioPacketTxControlBlock;
osMessageQId RadioPacketTxQueueHandle;

simple_link_packet_t ControlPacketBuffer[ 2 ];
osStaticMessageQDef_t ControlPacketControlBlock;
osMessageQId ControlPacketQueueHandle;

osStaticMutexDef_t SimpleLinkMutexControlBlock;
osMutexId SimpleLinkMutexHandle;

osThreadId 	tasks_ids[4];
uint32_t 	tasks_full_stack[4];

static simple_link_packet_t control_packet;
static comms_hk_data_t comms_data;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void ControlFunc(void const * argument);
void CommsFunc(void const * argument);
void CommsTxFunc(void const * argument);
void InterfaceFunc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
static void GetModuleHKData(comms_hk_data_t *data);
/* USER CODE END FunctionPrototypes */

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */ 
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexStaticDef(SimpleLinkMutex, &SimpleLinkMutexControlBlock);
  SimpleLinkMutexHandle = osMutexCreate(osMutex(SimpleLinkMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ControlTask */
  osThreadStaticDef(ControlTask, ControlFunc, osPriorityNormal, 0, 256, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of CommsTask */
  osThreadStaticDef(CommsTask, CommsFunc, osPriorityHigh, 0, 768, CommsBuffer, &CommsControlBlock);
  CommsTaskHandle = osThreadCreate(osThread(CommsTask), NULL);

  /* definition and creation of CommsTxTask */
  osThreadStaticDef(CommsTxTask, CommsTxFunc, osPriorityBelowNormal, 0, 256, CommsTxBuffer, &CommsTxControlBlock);
  CommsTxTaskHandle = osThreadCreate(osThread(CommsTxTask), NULL);

  /* definition and creation of InterfaceTask */
  osThreadStaticDef(InterfaceTask, InterfaceFunc, osPriorityHigh, 0, 256, InterfaceBuffer, &InterfaceControlBlock);
  InterfaceTaskHandle = osThreadCreate(osThread(InterfaceTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UartQueueTx */
  osMessageQStaticDef(UartQueueTx, 4096, uint8_t, UartQueueTxBuffer, &UartQueueTxControlBlock);
  UartQueueTxHandle = osMessageCreate(osMessageQ(UartQueueTx), NULL);

  /* definition and creation of UartQueueRx */
  osMessageQStaticDef(UartQueueRx, 4096, uint8_t, UartQueueRxBuffer, &UartQueueRxControlBlock);
  UartQueueRxHandle = osMessageCreate(osMessageQ(UartQueueRx), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  osMessageQStaticDef(linkrxqueue, 6, simple_link_packet_t, LinkLayerRxBuffer, &LinkLayerRxControlBlock);
  LinkLayerRxQueueHandle = osMessageCreate(osMessageQ(linkrxqueue), NULL);

  osMessageQStaticDef(radiotxqueue, 2, simple_link_packet_t, RadioPacketTxBuffer, &RadioPacketTxControlBlock);
  RadioPacketTxQueueHandle = osMessageCreate(osMessageQ(radiotxqueue), NULL);

  osMessageQStaticDef(controlqueue, 2, simple_link_packet_t, ControlPacketBuffer, &ControlPacketControlBlock);
  ControlPacketQueueHandle = osMessageCreate(osMessageQ(controlqueue), NULL);

  initialize_rs_coder();
  initialize_cc1101();
  usart_init_rx();
  usart_init_tx();
  /* USER CODE END RTOS_QUEUES */
}

/* ControlFunc function */
void ControlFunc(void const * argument)
{

  /* USER CODE BEGIN ControlFunc */
  /* Infinite loop */

	int ret;
	check_for_printf_buffer();

	tasks_ids[0] = ControlTaskHandle;
	tasks_ids[1] = CommsTaskHandle;
	tasks_ids[2] = CommsTxTaskHandle;
	tasks_ids[3] = InterfaceTaskHandle;

	tasks_full_stack[0] = (sizeof(ControlTaskBuffer)/sizeof(uint32_t));
	tasks_full_stack[1] = (sizeof(CommsBuffer)/sizeof(uint32_t));
	tasks_full_stack[2] = (sizeof(CommsTxBuffer)/sizeof(uint32_t));
	tasks_full_stack[3] = (sizeof(InterfaceBuffer)/sizeof(uint32_t));

	init_housekeeping();
	/* Just OSDelay here to set all variables up */
	osDelay(1);
	for(;;) {
		if (xQueueReceive(ControlPacketQueueHandle, &control_packet, 5000) == pdTRUE) {
			/* Control packet asks for GetModuleHKData */
			GetModuleHKData(&comms_data);
			ret = set_simple_link_packet(&comms_data, sizeof(comms_data), 1, 0, &control_packet);
			send_kiss_packet(0, &control_packet, ret);
		}
	}
  /* USER CODE END ControlFunc */
}

/* CommsFunc function */
void CommsFunc(void const * argument)
{
  /* USER CODE BEGIN CommsFunc */
  /* Infinite loop */
	check_for_printf_buffer();
    osDelay(1);
    cc1101_work();
  /* USER CODE END CommsFunc */
}

/* CommsTxFunc function */
void CommsTxFunc(void const * argument)
{
  /* USER CODE BEGIN CommsTxFunc */
  /* Infinite loop */
	check_for_printf_buffer();
	osDelay(1);
	csma_tx_work();
  /* USER CODE END CommsTxFunc */
}

/* InterfaceFunc function */
void InterfaceFunc(void const * argument)
{
  /* USER CODE BEGIN InterfaceFunc */
  /* Infinite loop */
	check_for_printf_buffer();
    osDelay(1);
    usart_work();
  /* USER CODE END InterfaceFunc */
}

/* USER CODE BEGIN Application */

void ReturnHKData(comms_hk_data_t *data)
{
	taskENTER_CRITICAL();
	memcpy(data, &comms_data, sizeof(comms_hk_data_t));
	taskEXIT_CRITICAL();
}

void GetModuleHKData(comms_hk_data_t *data)
{
	int i;
	cc1101_external_info_t cc1101_info;

	refresh_housekeeping();
	data->ext_temp = get_external_temperature();
	data->int_temp = get_internal_temperature();
	data->bus_volt = get_voltage();
	for (i = 0; i < 4; i++) {
			data->free_stack[i] = uxTaskGetStackHighWaterMark(tasks_ids[i]);
		}
	for (i = 0; i < 4; i++) {
		data->used_stack[i] = tasks_full_stack[i] - data->free_stack[i];
	}
	taskENTER_CRITICAL();
	data->ll_rx_packets = link_layer_info.decoded_packets;
	data->ll_tx_packets = link_layer_info.encoded_packets;
	get_cc1101_statistics(&cc1101_info);
	taskEXIT_CRITICAL();
	data->tx_remaining = uxQueueSpacesAvailable(RadioPacketTxQueueHandle);
	data->rx_queued = uxQueueMessagesWaiting(LinkLayerRxQueueHandle);
	data->phy_tx_failed_packets = cc1101_info.packet_not_tx_count;
	data->phy_rx_packets = cc1101_info.packet_rx_count;
	data->phy_rx_errors = cc1101_info.packet_errors_corrected;
	data->phy_tx_packets = cc1101_info.packet_tx_count;
	data->trx_status = cc1101_info.mode;
	data->last_rssi = cc1101_info.last_rssi;
	data->last_lqi = cc1101_info.last_lqi;
	data->actual_rssi = cc1101_info.actual_rssi;

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
