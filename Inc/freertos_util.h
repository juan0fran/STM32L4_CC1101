/*
 * freertos_util.h
 *
 *  Created on: 24 de may. de 2017
 *      Author: cubecat
 */

#ifndef FREERTOS_UTIL_H_
#define FREERTOS_UTIL_H_

#include "cmsis_os.h"

extern osThreadId ControlTaskHandle;
extern osThreadId CommsTaskHandle;
extern osThreadId InterfaceTaskHandle;

extern osThreadId cc1101TxHandle;
extern osThreadId UartTxHandle;

extern osThreadId GDOTaskHandle;

extern osMessageQId RadioPacketTxQueueHandle;
extern osMessageQId RadioPacketRxQueueHandle;

#define GDO_NOTIFY_GDO0				(1 << 0)
#define GDO_NOTIFY_GDO2				(1 << 1)

#define COMMS_NOTIFY_RESET  		(1 << 0)
#define COMMS_NOTIFY_SEND_REQ		(1 << 1)
#define COMMS_NOTITY_PHY_RECEIVED	(1 << 2)

#define IFACE_NOTIFY_TX_END			(1 << 0)

#endif /* FREERTOS_UTIL_H_ */
