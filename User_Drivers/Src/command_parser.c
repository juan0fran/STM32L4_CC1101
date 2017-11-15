#include "command_parser.h"

static cp_config_t command_parser_config;

static simple_link_packet_t rx_packet_buffer;
static uint8_t 				tx_packet_buffer[128];

static int process_command(simple_link_packet_t *packet)
{
	int ret;
	if (packet->fields.config1 == (uint8_t) data_frame) {
		/* Send notify to COMMS thread */
		if (xQueueSend(RadioPacketTxQueueHandle, packet, 0) != pdTRUE) {
			ret = set_simple_link_packet(NULL, 0, nack_frame, 0, packet);
			send_kiss_packet(0, packet, ret);
		}else {
			ret = set_simple_link_packet(NULL, 0, request_frame, 0, packet);
			send_kiss_packet(0, packet, ret);
		}
	}else if (packet->fields.config1 == configuration_frame) {
		if (xQueueSend(ControlPacketQueueHandle, packet, 0) != pdTRUE) {
			ret = set_simple_link_packet(NULL, 0, nack_frame, 0, packet);
			send_kiss_packet(0, packet, ret);
		}
	}else if (packet->fields.config1 == request_frame) {
		if (xQueueReceive(LinkLayerRxQueueHandle, packet, 0) == pdTRUE) {
			send_kiss_packet(0, packet, _ntohs(packet->fields.len) + SL_HEADER_SIZE);
		}else {
			ret = set_simple_link_packet(NULL, 0, nack_frame, 0, packet);
			send_kiss_packet(0, packet, ret);
		}
	}
	return 0;
}


void usart_work(void)
{
	simple_link_control_t s_control;
	osEvent event;
	uint32_t last_received_tick;
	volatile uint32_t messages_waiting;
	int32_t signals;
	uint8_t index;

	command_parser_config.reset_timeout = 1000;

	prepare_simple_link(&s_control);

	last_received_tick = osKernelSysTick();
	for(;;) {
		signals = IFACE_NOTIFY_RX | IFACE_NOTIFY_TX_REQ | IFACE_NOTIFY_ERROR;
		event = osSignalWait(signals, 1000);
		if (event.status == osEventSignal) {
			if (event.value.signals & IFACE_NOTIFY_ERROR) {
				usart_init_tx();
				usart_init_rx();
			}
			if (event.value.signals & IFACE_NOTIFY_RX) {
				if ( (uint32_t) (osKernelSysTick() - last_received_tick) > command_parser_config.reset_timeout) {
					prepare_simple_link(&s_control);
				}
				messages_waiting = osMessageWaiting(UartQueueRxHandle);
				while(messages_waiting > 0) {
					last_received_tick = osKernelSysTick();
					event = osMessageGet(UartQueueRxHandle, 0);
					if(get_simple_link_packet(event.value.v, &s_control, &rx_packet_buffer) > 0) {
						process_command(&rx_packet_buffer);
					}
					messages_waiting = osMessageWaiting(UartQueueRxHandle);
				}
			}
			if (event.value.signals & IFACE_NOTIFY_TX_REQ) {
				messages_waiting = osMessageWaiting(UartQueueTxHandle);
				if (messages_waiting > 0) {
					if (messages_waiting > (sizeof(tx_packet_buffer))) {
						messages_waiting = sizeof(tx_packet_buffer);
						osSignalSet(InterfaceTaskHandle, IFACE_NOTIFY_TX_REQ);
					}
					for (index = 0; index < messages_waiting; index++) {
						event = osMessageGet(UartQueueTxHandle, 0);
						if (event.status == osEventMessage) {
							tx_packet_buffer[index] = event.value.v;
						}else {
							break;
						}
					}
					_safe_send(tx_packet_buffer, index);
					signals = IFACE_NOTIFY_TX_END | IFACE_NOTIFY_ERROR;
					event = osSignalWait(signals, 100);
					if (event.status != osEventSignal) {
						usart_init_tx();
						usart_init_rx();
					}else {
						if (event.value.signals & IFACE_NOTIFY_ERROR) {
							usart_init_tx();
							usart_init_rx();
						}
					}
				}
			}
		}
	}
}
