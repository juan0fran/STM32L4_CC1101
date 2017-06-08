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
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId ControlTaskHandle;
uint32_t ControlTaskBuffer[ 1024 ];
osStaticThreadDef_t ControlTaskControlBlock;
osThreadId CommsTaskHandle;
uint32_t CommsTaskBuffer[ 4096 ];
osStaticThreadDef_t CommsTaskControlBlock;
osThreadId InterfaceTaskHandle;
uint32_t InterfaceTaskBuffer[ 2048 ];
osStaticThreadDef_t InterfaceTaskControlBlock;
osThreadId GDOTaskHandle;
uint32_t GDOTaskBuffer[ 680 ];
osStaticThreadDef_t GDOTaskControlBlock;

/* USER CODE BEGIN Variables */
radio_packet_t RadioPacketRxBuffer[ 16 ];
osStaticMessageQDef_t RadioPacketRxControlBlock;
osMessageQId RadioPacketRxQueueHandle;

simple_link_packet_t RadioPacketTxBuffer[ 4 ];
osStaticMessageQDef_t RadioPacketTxControlBlock;
osMessageQId RadioPacketTxQueueHandle;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void ControlFunc(void const * argument);
void CommsFunc(void const * argument);
void InterfaceFunc(void const * argument);
void GDOFunc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

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

  /* Create the mutex(es) */
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
  osThreadStaticDef(ControlTask, ControlFunc, osPriorityLow, 0, 1024, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of CommsTask */
  osThreadStaticDef(CommsTask, CommsFunc, osPriorityBelowNormal, 0, 4096, CommsTaskBuffer, &CommsTaskControlBlock);
  CommsTaskHandle = osThreadCreate(osThread(CommsTask), NULL);

  /* definition and creation of InterfaceTask */
  osThreadStaticDef(InterfaceTask, InterfaceFunc, osPriorityRealtime, 0, 2048, InterfaceTaskBuffer, &InterfaceTaskControlBlock);
  InterfaceTaskHandle = osThreadCreate(osThread(InterfaceTask), NULL);

  /* definition and creation of GDOTask */
  osThreadStaticDef(GDOTask, GDOFunc, osPriorityRealtime, 0, 680, GDOTaskBuffer, &GDOTaskControlBlock);
  GDOTaskHandle = osThreadCreate(osThread(GDOTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQStaticDef(radiorxqueue, 16, radio_packet_t, RadioPacketRxBuffer, &RadioPacketRxControlBlock);
  RadioPacketRxQueueHandle = osMessageCreate(osMessageQ(radiorxqueue), osThreadGetId());

  osMessageQStaticDef(radiotxqueue, 4, simple_link_packet_t, RadioPacketTxBuffer, &RadioPacketTxControlBlock);
  RadioPacketTxQueueHandle = osMessageCreate(osMessageQ(radiotxqueue), osThreadGetId());
  /* USER CODE END RTOS_QUEUES */
}

/* ControlFunc function */
void ControlFunc(void const * argument)
{

  /* USER CODE BEGIN ControlFunc */
  /* Infinite loop */
	uint32_t stack1, stack2, stack3, used1, used2, used3;
	for(;;) {
#if 1
		stack1 = uxTaskGetStackHighWaterMark(GDOTaskHandle);
		stack2 = uxTaskGetStackHighWaterMark(CommsTaskHandle);
		stack3 = uxTaskGetStackHighWaterMark(InterfaceTaskHandle);
		used1 = (sizeof(GDOTaskBuffer)/sizeof(uint32_t)) - stack1;
		used2 = (sizeof(CommsTaskBuffer)/sizeof(uint32_t)) - stack2;
		used3 = (sizeof(InterfaceTaskBuffer)/sizeof(uint32_t)) - stack3;
		print_uart_ln("Available GDO:\t\t%d words/ %d bytes", stack1, stack1*4);
		print_uart_ln("Available Comms:\t%d words/ %d bytes", stack2, stack2*4);
		print_uart_ln("Available Iface:\t%d words/ %d bytes", stack3, stack3*4);
		print_uart_ln("Used GDO:\t\t%d words/ %d bytes", used1, used1*4);
		print_uart_ln("Used Comms:\t\t%d words/ %d bytes", used2, used2*4);
		print_uart_ln("Used Iface:\t\t%d words/ %d bytes", used3, used3*4);
		print_uart_ln("TICK:\t\t\t%d", osKernelSysTick());
		print_uart_ln("");
#endif
		osDelay(5000);
	}
  /* USER CODE END ControlFunc */
}

/* CommsFunc function */
void CommsFunc(void const * argument)
{
  /* USER CODE BEGIN CommsFunc */
  /* Infinite loop */
	cc1101_work();
  /* USER CODE END CommsFunc */
}

/* InterfaceFunc function */
void InterfaceFunc(void const * argument)
{
  /* USER CODE BEGIN InterfaceFunc */
  /* Infinite loop */
	usart_work();
  /* USER CODE END InterfaceFunc */
}

/* GDOFunc function */
void GDOFunc(void const * argument)
{
  /* USER CODE BEGIN GDOFunc */
  /* Infinite loop */
	gdo_work();
  /* USER CODE END GDOFunc */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
