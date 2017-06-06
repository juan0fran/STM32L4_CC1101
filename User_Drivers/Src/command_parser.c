#include "command_parser.h"

osThreadId UsartRxHandle;
uint32_t UsartRxBuffer[ 1024 ];
osStaticThreadDef_t UsartRxControlBlock;

osThreadId UsartTxHandle;
uint32_t UsartTxBuffer[ 1024 ];
osStaticThreadDef_t UsartTxControlBlock;

static const int reset_timeout = 1000;

static int process_command(simple_link_packet_t *packet)
{
	if (packet->fields.config1 == (uint8_t) data_frame) {
		/* busy waiting till is finished or data storage? */

	}else {
		/* whatever */
	}
	return 0;
}

void usart_rx_work(void)
{
	simple_link_packet_t s_packet;
	simple_link_control_t s_control;
	osEvent event;
	uint32_t last_received_tick;
	int ret;

	usart_init_rx();
	prepare_simple_link(&s_control);

	last_received_tick = osKernelSysTick();
	for(;;) {
		event = osMessageGet(UartRxQueueHandle, osWaitForever);
		if (event.status == osEventMessage) {
			if ( (uint32_t) (osKernelSysTick() - last_received_tick) > reset_timeout) {
				prepare_simple_link(&s_control);
			}
			last_received_tick = osKernelSysTick();
			if( get_simple_link_packet(event.value.v, &s_control, &s_packet) > 0) {
				ret = set_simple_link_packet(s_packet.fields.payload, s_packet.fields.len, 0, 0, &s_packet);
				/* you can miss packets while sending this shit */
				send_kiss_packet(0, &s_packet, ret);
			}
		}
	}
}

void usart_tx_work(void const * argument)
{
	osEvent event;
	int32_t _signal;
	uint32_t messages_waiting;
	uint8_t burst_buffer[128];
	uint8_t index;
	usart_init_tx();
	for(;;) {
		event = osMessageGet(UartTxQueueHandle, osWaitForever);
		if (event.status == osEventMessage) {
			messages_waiting = osMessageWaiting(UartTxQueueHandle);
			if (messages_waiting > 0) {
				burst_buffer[0] = event.value.v;
				for (index = 1; index < messages_waiting; index++) {
					event = osMessageGet(UartTxQueueHandle, 0);
					if (event.status == osEventMessage) {
						burst_buffer[index] = event.value.v;
					}else {
						messages_waiting = index;
					}
				}
				_safe_send(burst_buffer, messages_waiting);
			}else {
				_safe_send(&event.value.v, 1);
			}
			_signal = IFACE_NOTIFY_TX_END;
			osSignalWait(_signal, osWaitForever);
			/* semaphore here to wait TX end */
		}
	}
}

void usart_work(void)
{
	  osThreadStaticDef(UsartTxWork, usart_tx_work, osPriorityNormal, 0, 1024, UsartTxBuffer, &UsartTxControlBlock);
	  UsartTxHandle = osThreadCreate(osThread(UsartTxWork), NULL);
	  usart_rx_work();
}
