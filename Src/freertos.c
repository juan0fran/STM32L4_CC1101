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
#include "housekeeping.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 8192 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId gdo0_taskHandle;
uint32_t myTask02Buffer[ 1024 ];
osStaticThreadDef_t myTask02ControlBlock;
osThreadId gdo2_taskHandle;
uint32_t myTask03Buffer[ 1024 ];
osStaticThreadDef_t myTask03ControlBlock;
osSemaphoreId gdo0_semHandle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
osSemaphoreId gdo2_semHandle;
osStaticSemaphoreDef_t myBinarySem02ControlBlock;

/* USER CODE BEGIN Variables */
static radio_parms_t radio;
static spi_parms_t spi;
static int i = 0, j = 0;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void gdo0_func(void const * argument);
void gdo2_func(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

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

  /* Create the semaphores(s) */
  /* definition and creation of gdo0_sem */
  osSemaphoreStaticDef(gdo0_sem, &myBinarySem01ControlBlock);
  gdo0_semHandle = osSemaphoreCreate(osSemaphore(gdo0_sem), 1);

  /* definition and creation of gdo2_sem */
  osSemaphoreStaticDef(gdo2_sem, &myBinarySem02ControlBlock);
  gdo2_semHandle = osSemaphoreCreate(osSemaphore(gdo2_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of gdo0_task */
  osThreadStaticDef(gdo0_task, gdo0_func, osPriorityHigh, 0, 1024, myTask02Buffer, &myTask02ControlBlock);
  gdo0_taskHandle = osThreadCreate(osThread(gdo0_task), NULL);

  /* definition and creation of gdo2_task */
  osThreadStaticDef(gdo2_task, gdo2_func, osPriorityHigh, 0, 1024, myTask03Buffer, &myTask03ControlBlock);
  gdo2_taskHandle = osThreadCreate(osThread(gdo2_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

	set_freq_parameters(434.92e6f, 433.92e6f, 384e3f, 2000.0f, &radio);
	set_sync_parameters(PREAMBLE_4, SYNC_30_over_32, 500, &radio);
	set_packet_parameters(false, true, &radio);
	set_modulation_parameters(RADIO_MOD_GFSK, RATE_9600, 0.5f, &radio);

	init_radio_config(&spi, &radio);
	enable_isr_routine(&spi, &radio);
	init_housekeeping();
	int temp_internal, temp_external, volt_bus;
  for(;;)
  {
	  refresh_housekeeping();
	  temp_internal = get_internal_temperature();
	  temp_external = get_external_temperature();
	  volt_bus = get_voltage();
	  taskENTER_CRITICAL();
	  print_uart_ln("Temp internal: %d C. Temp External: %d C. Bus voltage: %d mV. RSSI: %d dBm",
					  temp_internal, temp_external, volt_bus, (int) get_rssi());
	  print_uart_ln("i=%d,j=%d,r=%d", i, j, (int) get_rssi());
	  taskEXIT_CRITICAL();
	  osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* gdo0_func function */
void gdo0_func(void const * argument)
{
  /* USER CODE BEGIN gdo0_func */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(gdo0_semHandle, osWaitForever);
	  taskENTER_CRITICAL();
	  gdo0_isr();
	  taskEXIT_CRITICAL();
	  i++;
	  //osDelay(1);
  }
  /* USER CODE END gdo0_func */
}

/* gdo2_func function */
void gdo2_func(void const * argument)
{
  /* USER CODE BEGIN gdo2_func */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(gdo2_semHandle, osWaitForever);
	  taskENTER_CRITICAL();
	  gdo2_isr();
	  taskEXIT_CRITICAL();
	  j++;
	  //osDelay(1);
  }
  /* USER CODE END gdo2_func */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
