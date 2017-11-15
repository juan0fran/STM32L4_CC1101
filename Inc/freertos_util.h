/*
 * freertos_util.h
 *
 *  Created on: 24 de may. de 2017
 *      Author: cubecat
 */

#ifndef FREERTOS_UTIL_H_
#define FREERTOS_UTIL_H_

#include "cmsis_os.h"
#include "housekeeping.h"

/* Extern Task Handles */

/* Threads:
 * 	Control task
 * 	Interface task
 * 	Comms task
 * 	Comms tx task
 */

extern osThreadId ControlTaskHandle;

extern osThreadId InterfaceTaskHandle;

extern osThreadId CommsTaskHandle;
extern osThreadId CommsTxTaskHandle;

extern osMessageQId RadioPacketTxQueueHandle;
extern osMessageQId RadioPacketRxQueueHandle;

extern osMessageQId LinkLayerRxQueueHandle;

extern osMessageQId UartQueueRxHandle;
extern osMessageQId UartQueueTxHandle;

extern osMessageQId ControlPacketQueueHandle;

extern osMutexId SimpleLinkMutexHandle;

extern osThreadId 	tasks_ids[4];
extern uint32_t 	tasks_full_stack[4];

#define GDO_NOTIFY_GDO0				(1 << 0)
#define GDO_NOTIFY_GDO2				(1 << 1)
#define COMMS_NOTIFY_SEND_REQ		(1 << 2)
#define COMMS_NOTITY_PHY_RECEIVED	(1 << 3)
#define COMMS_NOTIFY_END_TX			(1 << 4)
#define COMMS_NOTIFY_RESET  		(1 << 5)

#define COMMS_WAIT_TIME				1000 /* ms */

#define IFACE_NOTIFY_TX_END			(1 << 0)
#define IFACE_NOTIFY_TX_REQ			(1 << 1)
#define IFACE_NOTIFY_RX				(1 << 2)
#define IFACE_NOTIFY_ERROR			(1 << 3)

#define CTRL_HK_DMA_END				(1 << 0)

typedef enum return_values_e {
	func_error 	= -1,
	func_ok		= 0,
	func_data 	= 1,
}return_values_t;

void ReturnHKData(comms_hk_data_t *data);

#endif /* FREERTOS_UTIL_H_ */
