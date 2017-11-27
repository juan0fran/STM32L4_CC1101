/***************************************************************************************************
*  File:        freertos_routines.c                                                                *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: Includes all FreeRTOS task initialization, definition and main task loop           *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

#include "freertos_routines.h"

TaskHandle_t    tasks_ids[4];
uint32_t        tasks_full_stack[4];

static simple_link_packet_t control_packet;
static comms_hk_data_t comms_data;

TaskHandle_t ControlTaskHandle;
uint32_t ControlTaskBuffer[ 256 ];
StaticTask_t ControlTaskControlBlock;

TaskHandle_t CommsTaskHandle;
uint32_t CommsBuffer[ 768 ];
StaticTask_t CommsControlBlock;

TaskHandle_t CommsTxTaskHandle;
uint32_t CommsTxBuffer[ 256 ];
StaticTask_t CommsTxControlBlock;

TaskHandle_t InterfaceTaskHandle;
uint32_t InterfaceBuffer[ 256 ];
StaticTask_t InterfaceControlBlock;

QueueHandle_t UartQueueTxHandle;
uint8_t UartQueueTxBuffer[ 4096 * sizeof( uint8_t ) ];
StaticQueue_t UartQueueTxControlBlock;

QueueHandle_t UartQueueRxHandle;
uint8_t UartQueueRxBuffer[ 4096 * sizeof( uint8_t ) ];
StaticQueue_t UartQueueRxControlBlock;

simple_link_packet_t LinkLayerRxBuffer[ 12 ];
StaticQueue_t LinkLayerRxControlBlock;
QueueHandle_t LinkLayerRxQueueHandle;

simple_link_packet_t RadioPacketTxBuffer[ 4 ];
StaticQueue_t RadioPacketTxControlBlock;
QueueHandle_t RadioPacketTxQueueHandle;

simple_link_packet_t ControlPacketBuffer[ 2 ];
StaticQueue_t ControlPacketControlBlock;
QueueHandle_t ControlPacketQueueHandle;

StaticSemaphore_t SimpleLinkMutexControlBlock;
SemaphoreHandle_t SimpleLinkMutexHandle;

void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize );

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
    data->housekeeping.ext_temp = convert_temp_f_u16(get_external_temperature());
    data->housekeeping.int_temp = convert_temp_f_u16(get_internal_temperature());
    for (i = 0; i < 4; i++) {
            data->control.free_stack[i] = uxTaskGetStackHighWaterMark(tasks_ids[i]);
        }
    for (i = 0; i < 4; i++) {
        data->control.used_stack[i] = tasks_full_stack[i] - data->control.free_stack[i];
    }
    taskENTER_CRITICAL();
    data->housekeeping.ll_rx_packets = link_layer_info.decoded_packets;
    data->housekeeping.ll_tx_packets = link_layer_info.encoded_packets;
    get_cc1101_statistics(&cc1101_info);
    taskEXIT_CRITICAL();

    data->control.tx_remaining = uxQueueSpacesAvailable(RadioPacketTxQueueHandle);
    data->control.rx_queued = uxQueueMessagesWaiting(LinkLayerRxQueueHandle);

    data->housekeeping.phy_tx_failed_packets = cc1101_info.packet_not_tx_count%65536;
    data->housekeeping.phy_rx_packets = cc1101_info.packet_rx_count;
    data->housekeeping.phy_rx_errors = cc1101_info.packet_errors_corrected%65536;
    data->housekeeping.phy_tx_packets = cc1101_info.packet_tx_count;

    data->housekeeping.last_rssi = cc1101_info.last_rssi;
    data->housekeeping.last_lqi = cc1101_info.last_lqi;
    data->housekeeping.actual_rssi = cc1101_info.actual_rssi;
    data->housekeeping.transmit_power = 0;

}


/* ControlFunc function */
void ControlFunc(void const * argument)
{

  /* USER CODE BEGIN 5 */
  int ret;
  check_for_printf_buffer();
  /* Infinite loop */
  for(;;)
  {
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
      vTaskDelay(1);
      for(;;) {
          if (xQueueReceive(ControlPacketQueueHandle, &control_packet, 5000) == pdTRUE) {
              /* Control packet asks for GetModuleHKData */
              GetModuleHKData(&comms_data);
              ret = set_simple_link_packet(&comms_data, sizeof(comms_data), configuration_frame, 0, &control_packet);
              send_kiss_packet(0, &control_packet, ret);
          }
      }
  }
  /* USER CODE END 5 */
}

/* CommsFunc function */
void CommsFunc(void const * argument)
{
  /* USER CODE BEGIN CommsFunc */
  /* Infinite loop */
  check_for_printf_buffer();
  for(;;)
  {
      vTaskDelay(1);
      cc1101_work();
  }
  /* USER CODE END CommsFunc */
}

/* CommsTxFunc function */
void CommsTxFunc(void const * argument)
{
  /* USER CODE BEGIN CommsTxFunc */
  /* Infinite loop */
  check_for_printf_buffer();
  for(;;)
  {
      vTaskDelay(1);
      csma_tx_work();
  }
  /* USER CODE END CommsTxFunc */
}

/* InterfaceFunc function */
void InterfaceFunc(void const * argument)
{
  /* USER CODE BEGIN InterfaceFunc */
  /* Infinite loop */
  check_for_printf_buffer();
  for(;;)
  {
      vTaskDelay(1);
      usart_work();
  }
  /* USER CODE END InterfaceFunc */
}


void init_freertos_tasks(void)
{
    /* add mutexes, ... */
    SimpleLinkMutexHandle = xSemaphoreCreateMutexStatic(&SimpleLinkMutexControlBlock);

    /* definition and creation of ControlTask */
    ControlTaskHandle = xTaskCreateStatic((void *)ControlFunc, "ControlTask",
                                        (sizeof(ControlTaskBuffer)/sizeof(ControlTaskBuffer[0])),
                                        NULL, 4, ControlTaskBuffer, &ControlTaskControlBlock);

    /* definition and creation of CommsTask */
    CommsTaskHandle = xTaskCreateStatic((void *)CommsFunc, "CommsTask",
                                        (sizeof(CommsBuffer)/sizeof(CommsBuffer[0])),
                                        NULL, 5, CommsBuffer, &CommsControlBlock);

    /* definition and creation of CommsTxTask */
    CommsTxTaskHandle = xTaskCreateStatic((void *)CommsTxFunc, "CommsTxTask",
                                        (sizeof(CommsTxBuffer)/sizeof(CommsTxBuffer[0])),
                                        NULL, 3, CommsTxBuffer, &CommsTxControlBlock);

    /* definition and creation of InterfaceTask */
    InterfaceTaskHandle = xTaskCreateStatic((void *)InterfaceFunc, "InterfaceTask",
                                        (sizeof(InterfaceBuffer)/sizeof(InterfaceBuffer[0])),
                                        NULL, 5, InterfaceBuffer, &InterfaceControlBlock);

    /* definition and creation of UartQueueTx */
    UartQueueTxHandle = xQueueCreateStatic(4096, sizeof(uint8_t), UartQueueTxBuffer, &UartQueueTxControlBlock);

    /* definition and creation of UartQueueRx */
    UartQueueRxHandle = xQueueCreateStatic(4096, sizeof(uint8_t), UartQueueRxBuffer, &UartQueueRxControlBlock);

    /* add queues, ... */
    LinkLayerRxQueueHandle = xQueueCreateStatic(12, sizeof(simple_link_packet_t),
                                          (uint8_t *) LinkLayerRxBuffer, &LinkLayerRxControlBlock);

    RadioPacketTxQueueHandle = xQueueCreateStatic(4, sizeof(simple_link_packet_t),
                                                          (uint8_t *) RadioPacketTxBuffer, &RadioPacketTxControlBlock);

    ControlPacketQueueHandle = xQueueCreateStatic(2, sizeof(simple_link_packet_t),
                                                          (uint8_t *) ControlPacketBuffer, &ControlPacketControlBlock);


    initialize_rs_coder();
    initialize_cc1101();
    usart_init_rx();
    usart_init_tx();
}
