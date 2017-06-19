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
uint32_t ControlTaskBuffer[ 1024 ];
osStaticThreadDef_t ControlTaskControlBlock;
osThreadId GDOTaskHandle;
uint32_t GDOTaskBuffer[ 1024 ];
osStaticThreadDef_t GDOTaskControlBlock;
osThreadId CommsRxTaskHandle;
uint32_t CommsRxBuffer[ 512 ];
osStaticThreadDef_t CommsRxControlBlock;
osThreadId CommsTxTaskHandle;
uint32_t CommsTxBuffer[ 512 ];
osStaticThreadDef_t CommsTxControlBlock;
osThreadId InterfaceRxTaskHandle;
uint32_t InterfaceRxBuffer[ 256 ];
osStaticThreadDef_t InterfaceRxControlBlock;
osThreadId InterfaceTxTaskHandle;
uint32_t InterfaceTxBuffer[ 256 ];
osStaticThreadDef_t InterfaceTxControlBlock;
osMessageQId UartQueueTxHandle;
uint8_t UartQueueTxBuffer[ 4096 * sizeof( uint8_t ) ];
osStaticMessageQDef_t UartQueueTxControlBlock;
osMessageQId UartQueueRxHandle;
uint8_t UartQueueRxBuffer[ 4096 * sizeof( uint8_t ) ];
osStaticMessageQDef_t UartQueueRxControlBlock;

/* USER CODE BEGIN Variables */
radio_packet_t RadioPacketRxBuffer[ 16 ];
osStaticMessageQDef_t RadioPacketRxControlBlock;
osMessageQId RadioPacketRxQueueHandle;

simple_link_packet_t RadioPacketTxBuffer[ 4 ];
osStaticMessageQDef_t RadioPacketTxControlBlock;
osMessageQId RadioPacketTxQueueHandle;

simple_link_packet_t ControlPacketBuffer[ 2 ];
osStaticMessageQDef_t ControlPacketControlBlock;
osMessageQId ControlPacketQueueHandle;

osThreadId 	tasks_ids[6];
uint32_t 	tasks_full_stack[6];

static simple_link_packet_t control_packet;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void ControlFunc(void const * argument);
void GDOFunc(void const * argument);
void CommsRxFunc(void const * argument);
void CommsTxFunc(void const * argument);
void InterfaceRxFunc(void const * argument);
void InterfaceTxFunc(void const * argument);

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
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ControlTask */
  osThreadStaticDef(ControlTask, ControlFunc, osPriorityNormal, 0, 1024, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of GDOTask */
  osThreadStaticDef(GDOTask, GDOFunc, osPriorityRealtime, 0, 1024, GDOTaskBuffer, &GDOTaskControlBlock);
  GDOTaskHandle = osThreadCreate(osThread(GDOTask), NULL);

  /* definition and creation of CommsRxTask */
  osThreadStaticDef(CommsRxTask, CommsRxFunc, osPriorityAboveNormal, 0, 512, CommsRxBuffer, &CommsRxControlBlock);
  CommsRxTaskHandle = osThreadCreate(osThread(CommsRxTask), NULL);

  /* definition and creation of CommsTxTask */
  osThreadStaticDef(CommsTxTask, CommsTxFunc, osPriorityAboveNormal, 0, 512, CommsTxBuffer, &CommsTxControlBlock);
  CommsTxTaskHandle = osThreadCreate(osThread(CommsTxTask), NULL);

  /* definition and creation of InterfaceRxTask */
  osThreadStaticDef(InterfaceRxTask, InterfaceRxFunc, osPriorityHigh, 0, 256, InterfaceRxBuffer, &InterfaceRxControlBlock);
  InterfaceRxTaskHandle = osThreadCreate(osThread(InterfaceRxTask), NULL);

  /* definition and creation of InterfaceTxTask */
  osThreadStaticDef(InterfaceTxTask, InterfaceTxFunc, osPriorityHigh, 0, 256, InterfaceTxBuffer, &InterfaceTxControlBlock);
  InterfaceTxTaskHandle = osThreadCreate(osThread(InterfaceTxTask), NULL);

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
  osMessageQStaticDef(radiorxqueue, 16, radio_packet_t, RadioPacketRxBuffer, &RadioPacketRxControlBlock);
  RadioPacketRxQueueHandle = osMessageCreate(osMessageQ(radiorxqueue), NULL);

  osMessageQStaticDef(radiotxqueue, 4, simple_link_packet_t, RadioPacketTxBuffer, &RadioPacketTxControlBlock);
  RadioPacketTxQueueHandle = osMessageCreate(osMessageQ(radiotxqueue), NULL);

  osMessageQStaticDef(controlqueue, 2, simple_link_packet_t, ControlPacketBuffer, &ControlPacketControlBlock);
  ControlPacketQueueHandle = osMessageCreate(osMessageQ(controlqueue), NULL);

  initialize_rs_coder();

  /* USER CODE END RTOS_QUEUES */
}

/* ControlFunc function */
void ControlFunc(void const * argument)
{

  /* USER CODE BEGIN ControlFunc */
  /* Infinite loop */
	comms_hk_data_t data;
	int ret;

	tasks_ids[0] = ControlTaskHandle;
	tasks_ids[1] = GDOTaskHandle;
	tasks_ids[2] = CommsRxTaskHandle;
	tasks_ids[3] = CommsTxTaskHandle;
	tasks_ids[4] = InterfaceRxTaskHandle;
	tasks_ids[5] = InterfaceTxTaskHandle;

	tasks_full_stack[0] = (sizeof(ControlTaskBuffer)/sizeof(uint32_t));
	tasks_full_stack[1] = (sizeof(GDOTaskBuffer)/sizeof(uint32_t));
	tasks_full_stack[2] = (sizeof(CommsRxBuffer)/sizeof(uint32_t));
	tasks_full_stack[3] = (sizeof(CommsTxBuffer)/sizeof(uint32_t));
	tasks_full_stack[4] = (sizeof(InterfaceRxBuffer)/sizeof(uint32_t));
	tasks_full_stack[5] = (sizeof(InterfaceTxBuffer)/sizeof(uint32_t));

	init_housekeeping();

	print_uart_ln("Starting Control Task");

	for(;;) {
		if (xQueueReceive(ControlPacketQueueHandle, &control_packet, 5000) == pdTRUE) {
			/* Control packet asks for GetModuleHKData */
			GetModuleHKData(&data);

			ret = set_simple_link_packet(&data, sizeof(data), 1, 0, &control_packet);
			send_kiss_packet(0, &control_packet, ret);
		}
#if 1
		//GetModuleHKData(&data);
		//print_uart_ln("");
#endif
		//osDelay(5000);
	}
  /* USER CODE END ControlFunc */
}

/* GDOFunc function */
void GDOFunc(void const * argument)
{
  /* USER CODE BEGIN GDOFunc */
  /* Infinite loop */
	print_uart_ln("Starting GDO Task");
	gdo_work();
  /* USER CODE END GDOFunc */
}

/* CommsRxFunc function */
void CommsRxFunc(void const * argument)
{
  /* USER CODE BEGIN CommsRxFunc */
  /* Infinite loop */
	print_uart_ln("Starting CC1101 RX Task");
	cc1101_rx_work();
  /* USER CODE END CommsRxFunc */
}

/* CommsTxFunc function */
void CommsTxFunc(void const * argument)
{
  /* USER CODE BEGIN CommsTxFunc */
  /* Infinite loop */
	print_uart_ln("Starting CC1101 TX Task");
	cc1101_tx_work();
  /* USER CODE END CommsTxFunc */
}

/* InterfaceRxFunc function */
void InterfaceRxFunc(void const * argument)
{
  /* USER CODE BEGIN InterfaceRxFunc */
  /* Infinite loop */
	print_uart_ln("Starting UART RX Task");
	usart_rx_work();
  /* USER CODE END InterfaceRxFunc */
}

/* InterfaceTxFunc function */
void InterfaceTxFunc(void const * argument)
{
  /* USER CODE BEGIN InterfaceTxFunc */
  /* Infinite loop */
	print_uart_ln("Starting UART TX Task");
	usart_tx_work();
  /* USER CODE END InterfaceTxFunc */
}

/* USER CODE BEGIN Application */
void GetModuleHKData(comms_hk_data_t *data)
{
	int i;
	cc1101_external_info_t cc1101_info;

	refresh_housekeeping();
	data->ext_temp = get_external_temperature();
	data->int_temp = get_internal_temperature();
	data->bus_volt = get_voltage();
	for (i = 0; i < 6; i++) {
			data->free_stack[i] = uxTaskGetStackHighWaterMark(tasks_ids[i]);
		}
	for (i = 0; i < 6; i++) {
		data->used_stack[i] = tasks_full_stack[i] - data->free_stack[i];
	}
	taskENTER_CRITICAL();
	data->ll_rx_packets = link_layer_info.decoded_packets;
	data->ll_tx_packets = link_layer_info.encoded_packets;
	get_cc1101_statistics(&cc1101_info);
	taskEXIT_CRITICAL();
	data->phy_tx_failed_packets = cc1101_info.packet_not_tx_count;
	data->phy_rx_packets = cc1101_info.packet_rx_count;
	data->phy_tx_packets = cc1101_info.packet_tx_count;
	data->trx_status = cc1101_info.mode;
	data->last_rssi = cc1101_info.last_rssi;
	data->last_lqi = cc1101_info.last_lqi;

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
