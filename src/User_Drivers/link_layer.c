/***************************************************************************************************
*  File:        link_layer.c                                                                       *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: Communication Layer Erasure Code implementation                                    *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/
#include "link_layer.h"

#define BIT_TO_POS_W4(x, y) ( (x&0x0F) << ((y*4)))
#define W4_POS_FROM_BIT(x, y) ( (x >> (y*4)) &0x0F)

link_layer_external_info_t link_layer_info;

int build_llc_packet(uint8_t *buffer, uint8_t size, llc_parms_t *parms, radio_packet_t *p)
{

	if (p == NULL || parms == NULL || buffer == NULL || size == 0 || size > MAC_PAYLOAD_SIZE){
		return -1;
	}
	if (parms->esi > (parms->k+parms->r)) {
		return -1;
	}

	memset(p->fields.data, 0, MAC_PAYLOAD_SIZE);
	memcpy(p->fields.data, buffer, size);
	p->fields.info_n_esi 		= 0 + BIT_TO_POS_W4(parms->esi, 0);
	p->fields.k_n_r 			= BIT_TO_POS_W4(parms->k, 1) + BIT_TO_POS_W4(parms->r, 0);
	p->fields.chunk_sequence 	= parms->chunk_seq;
	p->fields.source_address 	= parms->src_addr;
	return 0;

}

int get_llc_packet(radio_packet_t *p, llc_parms_t *parms)
{
	if (p == NULL || parms == NULL){
		return -1;
	}

	parms->k 			= W4_POS_FROM_BIT(p->fields.k_n_r, 1);
	parms->r 			= W4_POS_FROM_BIT(p->fields.k_n_r, 0);
	parms->esi 			= W4_POS_FROM_BIT(p->fields.info_n_esi, 0);
	parms->chunk_seq 	= p->fields.chunk_sequence;
	parms->src_addr 	= p->fields.source_address;

	if (parms->k > 16 || parms->k == 0) {
		return -1;
	}
	if (parms->r >= 16) {
		return -1;
	}
	if ((parms->k + parms->r) == 0) {
		return -1;
	}
	if (parms->esi > 16) {
		return -1;
	}
	return 0;
}

int init_chunk_handler(chunk_handler_t *handler)
{
	if (handler == NULL) {
		return -1;
	}
	link_layer_info.decoded_packets = 0;
	link_layer_info.encoded_packets = 0;
	/* Currently not receiving, toggled by someone out of the program */
	handler->last_chunk_time = 0;
	handler->current_chunk_count = 0;
	handler->current_sequence = 0;
	handler->last_sequence = 0;
	handler->module_initialised = false;
	handler->library_initialised = true;
	/* We just start it different, nothing has been received here  */
	if (of_rs_2_m_create_codec_instance(&handler->of_handler) == OF_STATUS_OK) {
		return 0;
	}else{
		handler->library_initialised = false;
		return -1;
	}
}

int set_new_packet_to_chunk(chunk_handler_t *handler, radio_packet_t *p, uint8_t *chunk)
{
	/* This gets the LLC PARMS */
	/* Once Set returns new chunk found, *chunk will contain the packet chunk! */
	uint32_t timeout;
	int i, idx;
	of_rs_2_m_parameters_t parms;

	if (handler == NULL || p == NULL || chunk == NULL){
		return -1;
	}

	if (! handler->library_initialised){
		init_chunk_handler(handler);
	}
    parms.encoding_symbol_length = MAC_PAYLOAD_SIZE;
	/* Get llc from previous packet */
	if (get_llc_packet(p, &handler->llc) != 0) {
		return -1;
	}
	timeout = xTaskGetTickCount() - handler->last_chunk_time;
	if ( (handler->current_sequence != handler->llc.chunk_seq) 	||
		 ((handler->module_initialised) == false)				||
		 (timeout > MAC_SEQUENCE_TIMEOUT_MS)) {

		if (handler->module_initialised == false){
			handler->last_sequence = handler->current_sequence - 1;
			handler->module_initialised = true;
		}
		//printf("New sequence arrived! %d\r\n", handler->llc.chunk_seq);
		parms.nb_repair_symbols = handler->llc.r;
		parms.nb_source_symbols = handler->llc.k;
		of_rs_2_m_set_fec_parameters(&handler->of_handler, &parms);
		handler->current_sequence = handler->llc.chunk_seq;
		if (handler->current_sequence == handler->last_sequence &&
			timeout > MAC_SEQUENCE_TIMEOUT_MS) {
			handler->last_sequence = handler->current_sequence - 1;
		}
		handler->current_chunk_count = 0;
		handler->last_chunk_time = xTaskGetTickCount();
		/* This is done just once to make sure capture all the packets! */
		/* And make the stuff */
	}
	if (handler->current_sequence == handler->last_sequence) {
		handler->last_chunk_time = xTaskGetTickCount();
		/* In case that last_seq_received is == to the current sequence, drop the packet,
		 * since the chunk is already received */
		//printf("Dropped packet with parameters K: %d, R: %d and ESI: %d\r\n", handler->llc.k, handler->llc.r, handler->llc.esi);
	}else{
		handler->last_chunk_time = xTaskGetTickCount();
		//printf("New packet has been received with parameters K: %d, R: %d and ESI: %d from Seq: %d\r\n", handler->llc.k, handler->llc.r, handler->llc.esi, handler->llc.chunk_seq);
		idx = handler->current_chunk_count * handler->of_handler.encoding_symbol_length;
		memcpy(&handler->chunk_reserved_memory[idx], p->fields.data, handler->of_handler.encoding_symbol_length);
		if (of_rs_2_m_decode_with_new_symbol(&handler->of_handler, &handler->chunk_reserved_memory[idx], handler->llc.esi) != OF_STATUS_OK) {
			//printf("[OPENFEC]: Error trying to decode\r\n");
			return 0;
		}
		handler->current_chunk_count++;
		if (handler->current_chunk_count == handler->llc.k && handler->of_handler.decoding_finished) {
			handler->last_sequence = handler->current_sequence;
			of_rs_2_m_get_source_symbols_tab(&handler->of_handler, handler->symb_tabs);
            //printf("[OPENFEC] get source symbols returned: %d\r\n", ret);
            for (i = 0; i < handler->llc.k; i++){
	            memcpy(chunk+i*handler->of_handler.encoding_symbol_length, handler->symb_tabs[i], handler->of_handler.encoding_symbol_length);
        	}
            link_layer_info.decoded_packets++;
        	return (handler->llc.k * handler->of_handler.encoding_symbol_length);
		}
	}
	return 0;
}

int get_new_packet_from_chunk(chunk_handler_t *handler, uint8_t *chunk, uint16_t size, uint8_t redundancy, radio_packet_t *p)
{
	of_rs_2_m_parameters_t parms;
	int packet_count;
	int idx;
	if (handler == NULL || p == NULL || chunk == NULL){
		return -1;
	}
	if (size == 0 || size > MAC_PAYLOAD_SIZE*(OF_MAX_ENCODING_SYMBOLS-redundancy)) {
		return -1;
	}
	if (! handler->library_initialised) {
		init_chunk_handler(handler);
	}
    parms.encoding_symbol_length = MAC_PAYLOAD_SIZE;
	/* Generates from a given chunk a set of packets and add redundancy on top of it! */
	if (handler->current_sequence != handler->last_sequence || !(handler->module_initialised)) {
		/* If the sequence is not the same, new chunk bro */
		packet_count = size/MAC_PAYLOAD_SIZE;
		if (size % MAC_PAYLOAD_SIZE != 0){
			packet_count++;
		}
		if (packet_count + redundancy > OF_MAX_ENCODING_SYMBOLS) {
			return -1;
		}
		if (handler->module_initialised == false){
			handler->last_sequence = handler->current_sequence - 1;
			handler->module_initialised = true;
		}
		handler->llc.k = (uint8_t) packet_count;
		handler->llc.r = redundancy;
		handler->llc.chunk_seq = handler->current_sequence;
		handler->llc.src_addr = 0;
		parms.nb_source_symbols = handler->llc.k;
		parms.nb_repair_symbols = handler->llc.r;

		/* Generate the FEC table */
		of_rs_2_m_set_fec_parameters(&handler->of_handler, &parms);
		memset(handler->chunk_reserved_memory, 0, sizeof(handler->chunk_reserved_memory));
		memcpy(handler->chunk_reserved_memory, chunk, size);
		handler->last_sequence = handler->current_sequence;
		handler->current_chunk_count = 0;
	}
	if (handler->current_sequence == handler->last_sequence) {
		/* The same sequence, just keep going:: DALE CALOR! */
		/* It will generate another radio_packet_t */
		idx = handler->current_chunk_count * handler->of_handler.encoding_symbol_length;
		handler->symb_tabs[handler->current_chunk_count] = &handler->chunk_reserved_memory[idx];
		of_rs_2_m_build_repair_symbol(&handler->of_handler, handler->symb_tabs, handler->current_chunk_count);
		handler->llc.esi = handler->current_chunk_count;
		build_llc_packet(handler->symb_tabs[handler->current_chunk_count], handler->of_handler.encoding_symbol_length, &handler->llc, p);
		handler->current_chunk_count++;
		/* Increment that counter and return */
		if (handler->current_chunk_count == (handler->llc.k + handler->llc.r)) {
			/* We are done, make something to not enter here again */
			link_layer_info.encoded_packets++;
			handler->current_sequence++;
		}
		/* while this functions > 0 -> keep setting it */
		return ((handler->llc.k + handler->llc.r) - handler->current_chunk_count);
	}
	return 0;
}
