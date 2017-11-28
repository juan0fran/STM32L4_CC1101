/***************************************************************************************************
*  File:        command_parser.c                                                                   *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: Command parser implementation, includes UART task main loop and command processing *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

#include "command_parser.h"

static cp_config_t command_parser_config;

static simple_link_packet_t rx_packet_buffer;
static uint8_t                 tx_packet_buffer[128];

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
    uint32_t last_received_tick;
    volatile uint32_t messages_waiting;
    uint32_t signals;
    uint32_t signal_received;
    uint8_t index;
    uint8_t received_byte;

    command_parser_config.reset_timeout = 1000;

    prepare_simple_link(&s_control);

    last_received_tick = xTaskGetTickCount();
    for(;;) {
        signals = IFACE_NOTIFY_RX | IFACE_NOTIFY_TX_REQ | IFACE_NOTIFY_ERROR;
        if(xTaskNotifyWait(0, signals, &signal_received, 1000/portTICK_PERIOD_MS) == pdTRUE) {
            if (signal_received & IFACE_NOTIFY_ERROR) {
                usart_init_tx();
                usart_init_rx();
            }
            if (signal_received & IFACE_NOTIFY_RX) {
                if ( (uint32_t) (xTaskGetTickCount() - last_received_tick) > command_parser_config.reset_timeout) {
                    prepare_simple_link(&s_control);
                }
                messages_waiting = uxQueueMessagesWaiting(UartQueueRxHandle);
                while(messages_waiting > 0) {
                    last_received_tick = xTaskGetTickCount();
                    xQueueReceive(UartQueueRxHandle, &received_byte, 0);
                    if(get_simple_link_packet(received_byte, &s_control, &rx_packet_buffer) > 0) {
                        process_command(&rx_packet_buffer);
                    }
                    messages_waiting = uxQueueMessagesWaiting(UartQueueRxHandle);
                }
            }
            if (signal_received & IFACE_NOTIFY_TX_REQ) {
                messages_waiting = uxQueueMessagesWaiting(UartQueueTxHandle);
                if (messages_waiting > 0) {
                    if (messages_waiting > (sizeof(tx_packet_buffer))) {
                        messages_waiting = sizeof(tx_packet_buffer);
                        xTaskNotify(InterfaceTaskHandle, IFACE_NOTIFY_TX_REQ, eSetBits);
                    }
                    for (index = 0; index < messages_waiting; index++) {
                        if(xQueueReceive(UartQueueTxHandle, &received_byte, 0) == pdTRUE) {
                            tx_packet_buffer[index] = received_byte;
                        }else {
                            break;
                        }
                    }
                    _safe_send(tx_packet_buffer, index);
                    signals = IFACE_NOTIFY_TX_END | IFACE_NOTIFY_ERROR;
                    if(xTaskNotifyWait(0, signals, &signal_received, 100/portTICK_PERIOD_MS) != pdTRUE) {
                        usart_init_tx();
                        usart_init_rx();
                    } else {
                        if (signal_received & IFACE_NOTIFY_ERROR) {
                            usart_init_tx();
                            usart_init_rx();
                        }
                    }
                }
            }
        }
        xTaskNotify(ControlTaskHandle, CTRL_WDT_INTERFACE_RESET, eSetBits);
    }
}
