#include "command_parser.h"

static cp_config_t command_parser_config;

static int cnt = 0;

static int process_command(simple_link_packet_t *packet)
{
	cnt++;
	print_uart_ln("Packet received, CNT: %d", cnt);
	if (packet->fields.config1 == (uint8_t) data_frame) {
		/* Send notify to COMMS thread */
		if (xQueueSend(RadioPacketTxQueueHandle, packet, 0) != pdTRUE) {
			print_uart_ln("You fucked it up!");
		}
	}else {

	}
	return 0;
}

void usart_rx_work(void)
{
	simple_link_packet_t s_packet;
	simple_link_control_t s_control;
	osEvent event;
	uint32_t last_received_tick;
	uint32_t messages_waiting;
	uint8_t index;
	int ret;
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
			last_received_tick = osKernelSysTick();
			do {
				event = osMessageGet(UartQueueRxHandle, 0);
				if(get_simple_link_packet(event.value.v, &s_control, &s_packet) > 0) {
					process_command(&s_packet);
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
	uint8_t burst_buffer[128];
	uint8_t index;
	usart_init_tx();
	for(;;) {
		event = osMessageGet(UartQueueTxHandle, osWaitForever);
		if (event.status == osEventMessage) {
			messages_waiting = osMessageWaiting(UartQueueTxHandle);
			if (messages_waiting > 0) {
				burst_buffer[0] = event.value.v;
				for (index = 1; index < messages_waiting; index++) {
					event = osMessageGet(UartQueueTxHandle, 0);
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
