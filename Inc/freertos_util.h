/*
 * freertos_util.h
 *
 *  Created on: 24 de may. de 2017
 *      Author: cubecat
 */

#ifndef FREERTOS_UTIL_H_
#define FREERTOS_UTIL_H_

#include "cmsis_os.h"

/* Extern Task Handles */
extern osThreadId ControlTaskHandle;

extern osThreadId InterfaceRxTaskHandle;
extern osThreadId InterfaceTxTaskHandle;

extern osThreadId CommsRxTaskHandle;
extern osThreadId CommsTxTaskHandle;

extern osThreadId GDOTaskHandle;

extern osMessageQId RadioPacketTxQueueHandle;
extern osMessageQId RadioPacketRxQueueHandle;

extern osMessageQId UartQueueRxHandle;
extern osMessageQId UartQueueTxHandle;

extern osMessageQId ControlPacketQueueHandle;

extern osMutexId SimpleLinkMutexHandle;

extern osThreadId 	tasks_ids[6];
extern uint32_t 	tasks_full_stack[6];

#define GDO_NOTIFY_GDO0				(1 << 0)
#define GDO_NOTIFY_GDO2				(1 << 1)
#define GDO_NOTIFY_TX				(1 << 2)

#define COMMS_NOTIFY_RESET  		(1 << 0)
#define COMMS_NOTIFY_SEND_REQ		(1 << 1)
#define COMMS_NOTITY_PHY_RECEIVED	(1 << 2)
#define COMMS_NOTIFY_END_TX			(1 << 3)

#define IFACE_NOTIFY_TX_END			(1 << 0)

#endif /* FREERTOS_UTIL_H_ */
