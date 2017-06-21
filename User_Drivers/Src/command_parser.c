#include "command_parser.h"

static cp_config_t command_parser_config;

//static int cnt = 0;

static simple_link_packet_t rx_packet_buffer;
static simple_link_packet_t control_packet;
static uint8_t 				tx_packet_buffer[128];

static int process_command(simple_link_packet_t *packet)
{
	int ret;
	if (packet->fields.config1 == (uint8_t) data_frame) {
		/* Send notify to COMMS thread */
		if (xQueueSend(RadioPacketTxQueueHandle, packet, 0) != pdTRUE) {
			ret = set_simple_link_packet(NULL, 0, 3, 0, &control_packet);
			send_kiss_packet(0, &control_packet, ret);
		}else {
			ret = set_simple_link_packet(NULL, 0, 2, 0, &control_packet);
			send_kiss_packet(0, &control_packet, ret);
		}
	}else {
		if (xQueueSend(ControlPacketQueueHandle, packet, 0) != pdTRUE) {
			ret = set_simple_link_packet(NULL, 0, 3, 0, &control_packet);
			send_kiss_packet(0, &control_packet, ret);
		}
	}
	return 0;
}

void usart_rx_work(void)
{
	simple_link_control_t s_control;
	osEvent event;
	uint32_t last_received_tick;
	uint32_t messages_waiting;

	command_parser_config.reset_timeout = 1000;
	usart_init_rx();
	prepare_simple_link(&s_control);

	last_received_tick = osKernelSysTick();
	for(;;) {
		event = osMessagePeek(UartQueueRxHandle, osWaitForever);
		if (event.status == osEventMessage) {
			if ( (uint32_t) (osKernelSysTick() - last_received_tick) > command_parser_config.reset_timeout) {
				prepare_simple_link(&s_control);
			}
			do {
				last_received_tick = osKernelSysTick();
				event = osMessageGet(UartQueueRxHandle, 0);
				if(get_simple_link_packet(event.value.v, &s_control, &rx_packet_buffer) > 0) {
					process_command(&rx_packet_buffer);
				}
				messages_waiting = osMessageWaiting(UartQueueRxHandle);
			}while (messages_waiting > 0);
		}
	}
}

void usart_tx_work(void)
{
	osEvent event;
	int32_t _signal;
	uint32_t messages_waiting;
	uint8_t index;
	usart_init_tx();
	for(;;) {
		event = osMessageGet(UartQueueTxHandle, osWaitForever);
		if (event.status == osEventMessage) {
			messages_waiting = osMessageWaiting(UartQueueTxHandle);
			if (messages_waiting > 0) {
				if (messages_waiting > (sizeof(tx_packet_buffer) - 1)) {
					messages_waiting = sizeof(tx_packet_buffer) - 1;
				}
				tx_packet_buffer[0] = event.value.v;
				for (index = 1; index < messages_waiting; index++) {
					event = osMessageGet(UartQueueTxHandle, 0);
					if (event.status == osEventMessage) {
						tx_packet_buffer[index] = event.value.v;
					}else {
						break;
					}
				}
				_safe_send(tx_packet_buffer, index);
			}else {
				_safe_send(&event.value.v, 1);
			}
			_signal = IFACE_NOTIFY_TX_END;
			osSignalWait(_signal, osWaitForever);
			/* semaphore here to wait TX end */
		}
	}
}
