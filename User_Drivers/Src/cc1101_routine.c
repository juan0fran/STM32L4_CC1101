#include "cc1101_routine.h"

#define TX_FIFO_REFILL 58 // With the default FIFO thresholds selected this is the number of bytes to refill the Tx FIFO
#define RX_FIFO_UNLOAD 59 // With the default FIFO thresholds selected this is the number of bytes to unload from the Rx FIFO

static radio_parms_t radio_parms;
static spi_parms_t spi_parms_it;
static radio_int_data_t radio_int_data;
static bool init_radio = false;

static radio_packet_t rx_radio_packet;

static uint8_t actual_rssi;

static const float chanbw_limits[] = {
    812000.0, 650000.0, 541000.0, 464000.0, 406000.0, 325000.0, 270000.0, 232000.0,
    203000.0, 162000.0, 135000.0, 116000.0, 102000.0, 81000.0, 68000.0, 58000.0
};

static const uint32_t rate_values[] = {
    50, 110, 300, 600, 1200, 2400, 4800, 9600,
    14400, 19200, 28800, 38400, 57600, 76800, 115200,
};

static const float freq_temp_sensibility = 87.992;
static const float freq_temp_offset = -782.45;

static int 		CC_SPIWriteReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t byte);
static int     	CC_SPIWriteBurstReg(spi_parms_t *spi_parms, uint8_t addr, const uint8_t *bytes, uint8_t count);
static int     	CC_SPIReadReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t *byte);
static int     	CC_SPIReadBurstReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t *bytes, uint8_t count);
static int     	CC_SPIReadStatus(spi_parms_t *spi_parms, uint8_t addr, uint8_t *status);
static int     	CC_SPIStrobe(spi_parms_t *spi_parms, uint8_t strobe);
static int     	CC_PowerupResetCCxxxx(spi_parms_t *spi_parms);

static void    	disable_IT(void);
static void    	enable_IT(void);

uint32_t 		get_freq_word(uint32_t freq_xtal, uint32_t freq_hz, int32_t freq_off);
static uint32_t get_if_word(uint32_t freq_xtal, uint32_t if_hz);
static uint8_t 	get_offset_word(uint32_t freq_xtal, int32_t offset_hz);

static void     get_chanbw_words(float bw, radio_parms_t *radio_parms);
static void     get_rate_words(rate_t data_rate, float mod_index, radio_parms_t *radio_parms);

static void     radio_turn_idle(spi_parms_t *spi_parms);
static void     radio_turn_rx_isr(spi_parms_t *spi_parms);
static void     radio_turn_rx(spi_parms_t *spi_parms);
static void		radio_turn_tx(spi_parms_t *spi_parms);

static uint8_t 	radio_csma(void);

static void     radio_flush_fifos(spi_parms_t *spi_parms);

static void 	reset_parameters(void);

static int 		set_freq(int _0_rx_1_tx_flag);

static void 	disable_lna_enable_pa(void);
static void 	disable_pa_enable_lna(void);

void get_cc1101_statistics(cc1101_external_info_t *cc1101_info)
{
	cc1101_info->last_lqi = rx_radio_packet.fields.lqi;
	cc1101_info->last_rssi = rx_radio_packet.fields.rssi;
	cc1101_info->mode = radio_int_data.mode;
	cc1101_info->packet_rx_count =  radio_int_data.packet_rx_count;
	cc1101_info->packet_errors_corrected = radio_int_data.packet_rx_corrected;
	cc1101_info->packet_tx_count = radio_int_data.packet_tx_count;
	cc1101_info->packet_not_tx_count = radio_int_data.packet_not_tx_count;
	cc1101_info->actual_rssi = actual_rssi;
}

static uint8_t get_dec_rssi()
{
	uint8_t status;
	CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_RSSI, &status);
	return status;
}

// ------------------------------------------------------------------------------------------------
// Processes packets up to 255 bytes
int gdo0_isr(void)
// ------------------------------------------------------------------------------------------------
{
    uint8_t int_line, status;
    int ret = func_ok;
    int corrected_errors;
    if (init_radio == false) {
        return func_error;
    }
    int_line = CC11xx_GDO0(); // Sense interrupt line to determine if it was a raising or falling edge

    if (radio_int_data.mode == RADIOMODE_RX) {
        if (int_line){         
        	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_IOCFG2, 0x00);
            radio_int_data.byte_index = 0;
            radio_int_data.rx_count = CC11xx_PACKET_COUNT_SIZE;
            radio_int_data.bytes_remaining = radio_int_data.rx_count;
            radio_int_data.packet_receive = 1; // reception is in progress
        }else{
            if (radio_int_data.packet_receive) {
                CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_RXBYTES, &status);
                if ( (status&0x80) == 0x80) { /* Overflow */
                    radio_turn_idle(radio_int_data.spi_parms);
                }else {
                    CC_SPIReadBurstReg(radio_int_data.spi_parms, CC11xx_RXFIFO, (uint8_t *) &(radio_int_data.rx_buf[radio_int_data.byte_index]), radio_int_data.bytes_remaining);
                    radio_int_data.byte_index += radio_int_data.bytes_remaining;
                    radio_int_data.bytes_remaining = 0;

                    /* get lqi and rssi */
                    CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_LQI, &status);
                    rx_radio_packet.fields.lqi = status&0x7F;

                    CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_RSSI, &status);
                    rx_radio_packet.fields.rssi = status;

                    if ( (corrected_errors = decode_rs_message((uint8_t *) radio_int_data.rx_buf, CC11xx_PACKET_COUNT_SIZE, rx_radio_packet.raw, MAC_UNCODED_PACKET_SIZE)) != -1) {
                    	radio_int_data.packet_rx_corrected += corrected_errors;
                    	radio_int_data.packet_rx_count++;
                    	ret = func_data;
                    }
			    }
                radio_turn_rx_isr(radio_int_data.spi_parms);
            }
        }    
    }else if (radio_int_data.mode == RADIOMODE_TX) {
        if (int_line){
        	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_IOCFG2, 0x02); // GDO2 output pin config TX mode
            radio_int_data.packet_send = 1; // Assert packet transmission after sync has been sent
        }else{
            if (radio_int_data.packet_send) {
                CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_TXBYTES, &status);
                if ( (status&0x80) == 0x80) { /* Underflow */
                	reset_parameters();
                    radio_turn_idle(radio_int_data.spi_parms);
                }else {
                	reset_parameters(); // De-assert packet transmission after packet has been sent
                    radio_int_data.packet_tx_count++;
                    if ((radio_int_data.bytes_remaining)) {
                        radio_turn_idle(radio_int_data.spi_parms);          
                    }
                }
            }
            /* notify comms tx it can transfer another packet */
            osSignalSet(CommsTxTaskHandle, COMMS_NOTIFY_END_TX);
            radio_turn_rx_isr(radio_int_data.spi_parms);
        }
    }
    return ret;
}

// ------------------------------------------------------------------------------------------------
// Processes packets that do not fit in Rx or Tx FIFOs and 255 bytes long maximum
// FIFO threshold interrupt handler 
void gdo2_isr(void)
// ------------------------------------------------------------------------------------------------
{
    uint8_t int_line, bytes_to_send;
    if (init_radio == false) {
        return;
    }
    int_line = CC11xx_GDO2(); // Sense interrupt line to determine if it was a raising or falling edge

    if ((radio_int_data.mode == RADIOMODE_RX) && (int_line)) {
        if (radio_int_data.packet_receive){
            /* if this shit wants to write but rx_buf will overload, just break */
            CC_SPIReadBurstReg(radio_int_data.spi_parms, CC11xx_RXFIFO, (uint8_t *) &(radio_int_data.rx_buf[radio_int_data.byte_index]), RX_FIFO_UNLOAD);
            /* Check for status byte in each */
            radio_int_data.byte_index += RX_FIFO_UNLOAD;
            radio_int_data.bytes_remaining -= RX_FIFO_UNLOAD;    
            return;        
        }
    }
    if ((radio_int_data.mode == RADIOMODE_TX) && (!int_line)) {
        if ((radio_int_data.packet_send) && (radio_int_data.bytes_remaining > 0)) {
            if (radio_int_data.bytes_remaining < TX_FIFO_REFILL){
                bytes_to_send = radio_int_data.bytes_remaining;
            }else{
                bytes_to_send = TX_FIFO_REFILL;
            }
            CC_SPIWriteBurstReg(radio_int_data.spi_parms, CC11xx_TXFIFO, (uint8_t *) &(radio_int_data.tx_buf[radio_int_data.byte_index]), bytes_to_send);

            /* Check for status byte in each */
            radio_int_data.byte_index += bytes_to_send;
            radio_int_data.bytes_remaining -= bytes_to_send;
            return;
        }
    }
}

int set_freq_parameters(float freq_rx, float freq_tx, float freq_if, float freq_off, radio_parms_t * radio_parms)
{
    radio_parms->freq_rx        = freq_rx;
    radio_parms->freq_tx        = freq_tx;
    radio_parms->f_if           = freq_if;
	radio_parms->f_off 			= freq_off;

    /* Set the nominal parameters */
    radio_parms->f_xtal        = 26000000;
    radio_parms->chanspc_m     = 0;                // Do not use channel spacing for the moment defaulting to 0
    radio_parms->chanspc_e     = 0;                // Do not use channel spacing for the moment defaulting to 0
	radio_parms->chanbw_e 	   = 3;
	radio_parms->chanbw_m 	   = 3;

	return 0;
}

int set_sync_parameters(preamble_t preamble, sync_word_t sync_word, uint32_t timeout_ms, radio_parms_t * radio_parms)
{
    radio_parms->preamble       = preamble;
    radio_parms->sync_ctl       = sync_word;
	radio_parms->timeout 		= timeout_ms;
	
	return 0;
}

int set_packet_parameters(bool fec, bool white, radio_parms_t * radio_parms)
{
	radio_parms->fec            = (uint8_t) fec;
    radio_parms->whitening      = (uint8_t) white;
		
	return 0;
}
int set_modulation_parameters(radio_modulation_t mod, rate_t data_rate, float mod_index, radio_parms_t * radio_parms)
{
    radio_parms->modulation     = mod;
    radio_parms->drate          = data_rate;
    radio_parms->mod_index      = mod_index;

	return 0;
}

int init_radio_config(spi_parms_t * spi_parms, radio_parms_t * radio_parms)
{
    uint8_t  reg_word;
    uint8_t  patable[8];
    uint8_t  index;
    // Write register settings
	if (spi_parms == NULL){
		return -1;
	}
	if (radio_parms == NULL){
		return -1;
	}

    if (CC_PowerupResetCCxxxx(spi_parms) != 0){
    	return -1;
    }

    /* Patable Write here? */
    /* First read it */

    CC_SPIReadBurstReg(spi_parms, CC11xx_PATABLE, patable, sizeof(patable));
    for (index = 0; index < sizeof(patable); index++) {
    	patable[index] = 0x88;
    }
    CC_SPIWriteBurstReg(spi_parms, CC11xx_PATABLE, patable, sizeof(patable));
    // IOCFG2 = 0x00: Set in Rx mode (0x02 for Tx mode)
    // o 0x00: Asserts when RX FIFO is filled at or above the RX FIFO threshold. 
    //         De-asserts when RX FIFO is drained below the same threshold.
    // o 0x02: Asserts when the TX FIFO is filled at or above the TX FIFO threshold.
    //         De-asserts when the TX FIFO is below the same threshold.
    // GDO2 changes depending on the mode
    CC_SPIWriteReg(spi_parms, CC11xx_IOCFG2,   0x00); // GDO2 output pin config.

    // IOCFG1 = 0x0E: Set in Carrier Sense mode
    CC_SPIWriteReg(spi_parms, CC11xx_IOCFG1,   0x0E); // GDO1 output pin config.

    // IOCFG0 = 0x06: Asserts when sync word has been sent / received, and de-asserts at the
    // end of the packet. In RX, the pin will de-assert when the optional address
    // check fails or the RX FIFO overflows. In TX the pin will de-assert if the TX
    // FIFO underflows:    
    // GDO0 never changes 
    CC_SPIWriteReg(spi_parms, CC11xx_IOCFG0,   0x06); // GDO0 output pin config.

    // FIFO_THR = 14: 
    // o 5 bytes in TX FIFO (55 available spaces)
    // o 60 bytes in the RX FIFO
    CC_SPIWriteReg(spi_parms, CC11xx_FIFOTHR,  0x0E); // FIFO threshold.

    // PKTLEN: packet length up to 255 bytes. 
    CC_SPIWriteReg(spi_parms, CC11xx_PKTLEN, CC11xx_PACKET_COUNT_SIZE); // Packet length.
    
    // PKTCTRL0: Packet automation control #0
    // . bit  7:   unused
    // . bit  6:   0  -> whitening off
    // . bits 5:4: 00 -> normal mode use FIFOs for Rx and Tx
    // . bit  3:   unused
    // . bit  2:   1  -> CRC enabled
    // . bits 1:0: xx -> Packet length mode. Set to FIXED at 255.
    // CRC enabled by default
    reg_word = (radio_parms->whitening<<6) + 0x00; /* + 0x04 */; /* No CRC */
    CC_SPIWriteReg(spi_parms, CC11xx_PKTCTRL0, reg_word); // Packet automation control.

    // PKTCTRL1: Packet automation control #1
    // . bits 7:5: 000 -> Preamble quality estimator threshold
    // . bit  4:   unused
    // . bit  3:   0   -> Automatic flush of Rx FIFO disabled (too many side constraints see doc)
    // . bit  2:   1   -> Append two status bytes to the payload (RSSI and LQI + CRC OK)
    // . bits 1:0: 00  -> No address check of received packets
    CC_SPIWriteReg(spi_parms, CC11xx_PKTCTRL1, 0x00); // Packet automation control.

    CC_SPIWriteReg(spi_parms, CC11xx_ADDR,     0x00); // Device address for packet filtration (unused, see just above).
    CC_SPIWriteReg(spi_parms, CC11xx_CHANNR,   0x00); // Channel number (unused, use direct frequency programming).

    // FSCTRL0: Frequency offset added to the base frequency before being used by the
    // frequency synthesizer. (2s-complement). Multiplied by Fxtal/2^14
    CC_SPIWriteReg(spi_parms, CC11xx_FSCTRL0,  0x00); // Freq synthesizer control.

    // FSCTRL1: The desired IF frequency to employ in RX. Subtracted from FS base frequency
    // in RX and controls the digital complex mixer in the demodulator. Multiplied by Fxtal/2^10
    // Here 0.3046875 MHz (lowest point below 310 kHz)
    radio_parms->if_word = get_if_word(radio_parms->f_xtal, radio_parms->f_if);
    CC_SPIWriteReg(spi_parms, CC11xx_FSCTRL1, (radio_parms->if_word & 0x1F)); // Freq synthesizer control.

    // FREQ2..0: Base frequency for the frequency sythesizer
    // Fo = (Fxosc / 2^16) * FREQ[23..0]
    // FREQ2 is FREQ[23..16]
    // FREQ1 is FREQ[15..8]
    // FREQ0 is FREQ[7..0]
    // Fxtal = 26 MHz and FREQ = 0x10A762 => Fo = 432.99981689453125 MHz
    radio_parms->freq_word = get_freq_word(radio_parms->f_xtal, (uint32_t) radio_parms->freq_rx, (int32_t) radio_parms->f_off);
    CC_SPIWriteReg(spi_parms, CC11xx_FREQ2,    ((radio_parms->freq_word>>16) & 0xFF)); // Freq control word, high byte
    CC_SPIWriteReg(spi_parms, CC11xx_FREQ1,    ((radio_parms->freq_word>>8)  & 0xFF)); // Freq control word, mid byte.
    CC_SPIWriteReg(spi_parms, CC11xx_FREQ0,    (radio_parms->freq_word & 0xFF));       // Freq control word, low byte.

    get_rate_words(radio_parms->drate, radio_parms->mod_index, radio_parms);
    // MODCFG4 Modem configuration - bandwidth and data rate exponent
    // High nibble: Sets the decimation ratio for the delta-sigma ADC input stream hence the channel bandwidth
    // . bits 7:6: 0  -> CHANBW_E: exponent parameter (see next)
    // . bits 5:4: 2  -> CHANBW_M: mantissa parameter as per:
    //      BW = Fxosc / 8(4+CHANBW_M).2^CHANBW_E => Here: BW = 26/48 MHz = 541.67 kHz
    //      Factory defaults: M=0, E=1 => BW = 26/128 ~ 203 kHz
    // Low nibble:
    // . bits 3:0: 13 -> DRATE_E: data rate base 2 exponent => here 13 (multiply by 8192)
    reg_word = (radio_parms->chanbw_e<<6) + (radio_parms->chanbw_m<<4) + radio_parms->drate_e;
    CC_SPIWriteReg(spi_parms, CC11xx_MDMCFG4,  reg_word); // Modem configuration.

    // MODCFG3 Modem configuration: DRATE_M data rate mantissa as per formula:
    //    Rate = (256 + DRATE_M).2^DRATE_E.Fxosc / 2^28 
    // Here DRATE_M = 59, DRATE_E = 13 => Rate = 250 kBaud
    CC_SPIWriteReg(spi_parms, CC11xx_MDMCFG3,  radio_parms->drate_m); // Modem configuration.

    // MODCFG2 Modem configuration: DC block, modulation, Manchester, sync word
    // o bit 7:    0   -> Enable DC blocking (1: disable)
    // o bits 6:4: xxx -> (provided)
    // o bit 3:    0   -> Manchester disabled (1: enable)
    // o bits 2:0: 011 -> Sync word qualifier is 30/32 (static init in radio interface)
    reg_word = ((radio_parms->modulation)<<4) + radio_parms->sync_ctl;
    CC_SPIWriteReg(spi_parms, CC11xx_MDMCFG2,  reg_word); // Modem configuration.

    // MODCFG1 Modem configuration: FEC, Preamble, exponent for channel spacing
    // o bit 7:    0   -> FEC disabled (1: enable)
    // o bits 6:4: 2   -> number of preamble bytes (0:2, 1:3, 2:4, 3:6, 4:8, 5:12, 6:16, 7:24)
    // o bits 3:2: unused
    // o bits 1:0: CHANSPC_E: exponent of channel spacing (here: 2)
    reg_word = (radio_parms->fec<<7) + ((radio_parms->preamble)<<4) + (radio_parms->chanspc_e);
    CC_SPIWriteReg(spi_parms, CC11xx_MDMCFG1,  reg_word); // Modem configuration.

    // MODCFG0 Modem configuration: CHANSPC_M: mantissa of channel spacing following this formula:
    //    Df = (Fxosc / 2^18) * (256 + CHANSPC_M) * 2^CHANSPC_E
    //    Here: (26 /  ) * 2016 = 0.199951171875 MHz (200 kHz)
    CC_SPIWriteReg(spi_parms, CC11xx_MDMCFG0,  radio_parms->chanspc_m); // Modem configuration.

    // DEVIATN: Modem deviation
    // o bit 7:    0   -> not used
    // o bits 6:4: 0   -> DEVIATION_E: deviation exponent
    // o bit 3:    0   -> not used
    // o bits 2:0: 0   -> DEVIATION_M: deviation mantissa
    //
    //   Modulation  Formula
    //
    //   2-FSK    |  
    //   4-FSK    :  Df = (Fxosc / 2^17) * (8 + DEVIATION_M) * 2^DEVIATION_E : Here: 1.5869140625 kHz
    //   GFSK     |
    //
    //   MSK      :  Tx: not well documented, Rx: no effect
    //
    //   OOK      : No effect
    //    
    reg_word = (radio_parms->deviat_e<<4) + (radio_parms->deviat_m);
    CC_SPIWriteReg(spi_parms, CC11xx_DEVIATN,  reg_word); // Modem dev (when FSK mod en)

    // MCSM2: Main Radio State Machine. See documentation.
    //CC_SPIWriteReg(spi_parms, CC11xx_MCSM2 ,   0x00); //MainRadio Cntrl State Machine

    // MCSM1: Main Radio State Machine. 
    // o bits 7:6: not used
    // o bits 5:4: CCA_MODE: Clear Channel Indicator 
    //   0 (00): Always clear
    //   1 (01): Clear if RSSI below threshold
    //   2 (10): Always claar unless receiving a packet
    //   3 (11): Claar if RSSI below threshold unless receiving a packet
    // o bits 3:2: RXOFF_MODE: Select to what state it should go when a packet has been received
    //   0 (00): IDLE <== 
    //   1 (01): FSTXON
    //   2 (10): TX
    //   3 (11): RX (stay)
    // o bits 1:0: TXOFF_MODE: Select what should happen when a packet has been sent
    //   0 (00): IDLE <==
    //   1 (01): FSTXON
    //   2 (10): TX (stay)
    //   3 (11): RX 
    CC_SPIWriteReg(spi_parms, CC11xx_MCSM1 ,   0x00); //MainRadio Cntrl State Machine

    // MCSM0: Main Radio State Machine.
    // o bits 7:6: not used
    // o bits 5:4: FS_AUTOCAL: When to perform automatic calibration
    //   0 (00): Never i.e. manually via strobe command
    //   1 (01): When going from IDLE to RX or TX (or FSTXON)
    //   2 (10): When going from RX or TX back to IDLE automatically
    //   3 (11): Every 4th time when going from RX or TX to IDLE automatically
    // o bits 3:2: PO_TIMEOUT: 
    //   Value : Exp: Timeout after XOSC start
    //   0 (00):   1: Approx. 2.3 – 2.4 μs
    //   1 (01):  16: Approx. 37 – 39 μs
    //   2 (10):  64: Approx. 149 – 155 μs
    //   3 (11): 256: Approx. 597 – 620 μs
    // o bit 1: PIN_CTRL_EN:   Enables the pin radio control option
    // o bit 0: XOSC_FORCE_ON: Force the XOSC to stay on in the SLEEP state.
    CC_SPIWriteReg(spi_parms, CC11xx_MCSM0 ,   0x18); //MainRadio Cntrl State Machine

    // FOCCFG: Frequency Offset Compensation Configuration.
    // o bits 7:6: not used
    // o bit 5:    If set, the demodulator freezes the frequency offset compensation and clock
    //             recovery feedback loops until the CS signal goes high.
    // o bits 4:3: The frequency compensation loop gain to be used before a sync word is detected.
    //   0 (00): K
    //   1 (01): 2K
    //   2 (10): 3K
    //   3 (11): 4K
    // o bit 2: FOC_POST_K: The frequency compensation loop gain to be used after a sync word is detected.
    //   0: Same as FOC_PRE_K
    //   1: K/2
    // o bits 1:0: FOC_LIMIT: The saturation point for the frequency offset compensation algorithm:
    //   0 (00): ±0 (no frequency offset compensation)
    //   1 (01): ±BW CHAN /8
    //   2 (10): ±BW CHAN /4
    //   3 (11): ±BW CHAN /2
    CC_SPIWriteReg(spi_parms, CC11xx_FOCCFG,   0x1D); // Freq Offset Compens. Config

    // BSCFG:Bit Synchronization Configuration
    // o bits 7:6: BS_PRE_KI: Clock recovery loop integral gain before sync word
    //   0 (00): Ki
    //   1 (01): 2Ki
    //   2 (10): 3Ki
    //   3 (11): 4Ki
    // o bits 5:4: BS_PRE_KP: Clock recovery loop proportional gain before sync word
    //   0 (00): Kp
    //   1 (01): 2Kp
    //   2 (10): 3Kp
    //   3 (11): 4Kp
    // o bit 3: BS_POST_KI: Clock recovery loop integral gain after sync word
    //   0: Same as BS_PRE_KI
    //   1: Ki/2
    // o bit 2: BS_POST_KP: Clock recovery loop proportional gain after sync word
    //   0: Same as BS_PRE_KP
    //   1: Kp
    // o bits 1:0: BS_LIMIT: Data rate offset saturation (max data rate difference)
    //   0 (00): ±0 (No data rate offset compensation performed)
    //   1 (01): ±3.125 % data rate offset
    //   2 (10): ±6.25 % data rate offset
    //   3 (11): ±12.5 % data rate offset
    CC_SPIWriteReg(spi_parms, CC11xx_BSCFG,    0xB1); //  Bit synchronization config.

    // AGCCTRL2: AGC Control
    // o bits 7:6: MAX_DVGA_GAIN. Allowable DVGA settings
    //   0 (00): All gain settings can be used
    //   1 (01): The highest gain setting can not be used
    //   2 (10): The 2 highest gain settings can not be used
    //   3 (11): The 3 highest gain settings can not be used
    // o bits 5:3: MAX_LNA_GAIN. Maximum allowable LNA + LNA 2 gain relative to the maximum possible gain.
    //   0 (000): Maximum possible LNA + LNA 2 gain
    //   1 (001): Approx. 2.6 dB below maximum possible gain
    //   2 (010): Approx. 6.1 dB below maximum possible gain
    //   3 (011): Approx. 7.4 dB below maximum possible gain
    //   4 (100): Approx. 9.2 dB below maximum possible gain
    //   5 (101): Approx. 11.5 dB below maximum possible gain
    //   6 (110): Approx. 14.6 dB below maximum possible gain
    //   7 (111): Approx. 17.1 dB below maximum possible gain
    // o bits 2:0: MAGN_TARGET: target value for the averaged amplitude from the digital channel filter (1 LSB = 0 dB).
    //   0 (000): 24 dB
    //   1 (001): 27 dB
    //   2 (010): 30 dB
    //   3 (011): 33 dB
    //   4 (100): 36 dB
    //   5 (101): 38 dB
    //   6 (110): 40 dB
    //   7 (111): 42 dB
    //CC_SPIWriteReg(spi_parms, CC11xx_AGCCTRL2, 0xBB); // AGC control.
    CC_SPIWriteReg(spi_parms, CC11xx_AGCCTRL2, 0x4B); // AGC control.

    // AGCCTRL1: AGC Control
    // o bit 7: not used
    // o bit 6: AGC_LNA_PRIORITY: Selects between two different strategies for LNA and LNA 2 gain
    //   0: the LNA 2 gain is decreased to minimum before decreasing LNA gain
    //   1: the LNA gain is decreased first.
    // o bits 5:4: CARRIER_SENSE_REL_THR: Sets the relative change threshold for asserting carrier sense
    //   0 (00): Relative carrier sense threshold disabled
    //   1 (01): 6 dB increase in RSSI value
    //   2 (10): 10 dB increase in RSSI value
    //   3 (11): 14 dB increase in RSSI value
    // o bits 3:0: CARRIER_SENSE_ABS_THR: Sets the absolute RSSI threshold for asserting carrier sense. 
    //   The 2-complement signed threshold is programmed in steps of 1 dB and is relative to the MAGN_TARGET setting.
    //   0 is at MAGN_TARGET setting.
    CC_SPIWriteReg(spi_parms, CC11xx_AGCCTRL1, 0x40); // AGC control.

    // AGCCTRL0: AGC Control
    // o bits 7:6: HYST_LEVEL: Sets the level of hysteresis on the magnitude deviation
    //   0 (00): No hysteresis, small symmetric dead zone, high gain
    //   1 (01): Low hysteresis, small asymmetric dead zone, medium gain
    //   2 (10): Medium hysteresis, medium asymmetric dead zone, medium gain
    //   3 (11): Large hysteresis, large asymmetric dead zone, low gain
    // o bits 5:4: WAIT_TIME: Sets the number of channel filter samples from a gain adjustment has
    //   been made until the AGC algorithm starts accumulating new samples.
    //   0 (00):  8
    //   1 (01): 16
    //   2 (10): 24
    //   3 (11): 32
    // o bits 3:2: AGC_FREEZE: Control when the AGC gain should be frozen.
    //   0 (00): Normal operation. Always adjust gain when required.
    //   1 (01): The gain setting is frozen when a sync word has been found.
    //   2 (10): Manually freeze the analog gain setting and continue to adjust the digital gain.
    //   3 (11): Manually freezes both the analog and the digital gain setting. Used for manually overriding the gain.
    // o bits 0:1: FILTER_LENGTH: 
    //   2-FSK, 4-FSK, MSK: Sets the averaging length for the amplitude from the channel filter.    |  
    //   ASK ,OOK: Sets the OOK/ASK decision boundary for OOK/ASK reception.
    //   Value : #samples: OOK/ASK decision boundary
    //   0 (00):        8: 4 dB
    //   1 (01):       16: 8 dB
    //   2 (10):       32: 12 dB
    //   3 (11):       64: 16 dB  
    //CC_SPIWriteReg(spi_parms, CC11xx_AGCCTRL0, 0x83); // AGC control.

    // FREND1: Front End RX Configuration
    // o bits 7:6: LNA_CURRENT: Adjusts front-end LNA PTAT current output
    // o bits 5:4: LNA2MIX_CURRENT: Adjusts front-end PTAT outputs
    // o bits 3:2: LODIV_BUF_CURRENT_RX: Adjusts current in RX LO buffer (LO input to mixer)
    // o bits 1:0: MIX_CURRENT: Adjusts current in mixer
    CC_SPIWriteReg(spi_parms, CC11xx_FREND1,   0x56); // Front end RX configuration.

    // FREND0: Front End TX Configuration
    // o bits 7:6: not used
    // o bits 5:4: LODIV_BUF_CURRENT_TX: Adjusts current TX LO buffer (input to PA). The value to use
    //   in this field is given by the SmartRF Studio software
    // o bit 3: not used
    // o bits 1:0: PA_POWER: Selects PA power setting. This value is an index to the PATABLE, 
    //   which can be programmed with up to 8 different PA settings. In OOK/ASK mode, this selects the PATABLE
    //   index to use when transmitting a ‘1’. PATABLE index zero is used in OOK/ASK when transmitting a ‘0’. 
    //   The PATABLE settings from index ‘0’ to the PA_POWER value are used for ASK TX shaping, 
    //   and for power ramp-up/ramp-down at the start/end of transmission in all TX modulation formats.
    CC_SPIWriteReg(spi_parms, CC11xx_FREND0,   0x10); // Front end RX configuration.

    // FSCAL3: Frequency Synthesizer Calibration
    // o bits 7:6: The value to write in this field before calibration is given by the SmartRF
    //   Studio software.
    // o bits 5:4: CHP_CURR_CAL_EN: Disable charge pump calibration stage when 0.
    // o bits 3:0: FSCAL3: Frequency synthesizer calibration result register.
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL3,   0xE9); // Frequency synthesizer cal.

    // FSCAL2: Frequency Synthesizer Calibration
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL2,   0x2A); // Frequency synthesizer cal.
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL1,   0x00); // Frequency synthesizer cal.
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL0,   0x1F); // Frequency synthesizer cal.

    // TEST2: Various test settings. The value to write in this field is given by the SmartRF Studio software.
    CC_SPIWriteReg(spi_parms, CC11xx_TEST2,    0x81); // Various test settings.

    // TEST1: Various test settings. The value to write in this field is given by the SmartRF Studio software.
    CC_SPIWriteReg(spi_parms, CC11xx_TEST1,    0x35); // Various test settings.

    // TEST0: Various test settings. The value to write in this field is given by the SmartRF Studio software.
    CC_SPIWriteReg(spi_parms, CC11xx_TEST0,    0x09); // Various test settings.

    return 0;
}

int reconfigure_radio_config(spi_parms_t * spi_parms, radio_parms_t * radio_parms)
{
	return init_radio_config(spi_parms, radio_parms);
}

int set_freq(int _0_rx_1_tx_flag)
{
	if (_0_rx_1_tx_flag == 0) {
		radio_int_data.radio_parms->freq_word = get_freq_word(radio_int_data.radio_parms->f_xtal,
															  (uint32_t) radio_int_data.radio_parms->freq_rx,
															  (int32_t) radio_int_data.radio_parms->f_off);
	}else {
		radio_int_data.radio_parms->freq_word = get_freq_word(radio_int_data.radio_parms->f_xtal,
															  (uint32_t) radio_int_data.radio_parms->freq_tx,
															  (int32_t) radio_int_data.radio_parms->f_off);
	}
    CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_FREQ2,    ((radio_int_data.radio_parms->freq_word>>16) & 0xFF)); // Freq control word, high byte
    CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_FREQ1,    ((radio_int_data.radio_parms->freq_word>>8)  & 0xFF)); // Freq control word, mid byte.
    CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_FREQ0,    (radio_int_data.radio_parms->freq_word & 0xFF));       // Freq control word, low byte.
    return 0;
}

// ------------------------------------------------------------------------------------------------
// Calculate RSSI in dBm from decimal RSSI read out of RSSI status register
float rssi_raw_dbm(uint8_t rssi_dec)
// ------------------------------------------------------------------------------------------------
{
    if (rssi_dec < 128)
    {
        return (rssi_dec / 2.0) - 74.0;
    }
    else
    {
        return ((rssi_dec - 256) / 2.0) - 74.0;
    }
}

// ------------------------------------------------------------------------------------------------
// Calculate RSSI in dBm from decimal RSSI read out of RSSI status register
float rssi_lna_dbm(uint8_t rssi_dec)
// ------------------------------------------------------------------------------------------------
{
    if (rssi_dec < 128)
    {
    	return (rssi_dec / 2.0) - 89.9;
    }
    else
    {
    	return ((rssi_dec - 256) / 2.0) - 89.9;
    }
}

float lqi_status(uint8_t lqi)
{
	return (1.0 - ((float) lqi/127.0)) * 100.0;
}

// ------------------------------------------------------------------------------------------------
// Calculate frequency word FREQ[23..0]
uint32_t get_freq_word(uint32_t freq_xtal, uint32_t freq_hz, int32_t freq_off)
// ------------------------------------------------------------------------------------------------
{
    uint64_t res; // calculate on 64 bits to save precision
    res = ((uint64_t) (freq_hz+freq_off) * (uint64_t) (1<<16)) / ((uint64_t) freq_xtal);
    return (uint32_t) res;
}

// ------------------------------------------------------------------------------------------------
// Calculate frequency word FREQ[23..0]
uint32_t get_if_word(uint32_t freq_xtal, uint32_t if_hz)
// ------------------------------------------------------------------------------------------------
{
    return (if_hz * (1<<10)) / freq_xtal;
}

uint8_t get_offset_word(uint32_t freq_xtal, int32_t offset_hz)
{
		return ((offset_hz * (1 << 14)) / freq_xtal) & 0xFF;
}

// ------------------------------------------------------------------------------------------------
// Calculate CHANBW words according to CC1101 bandwidth steps
void get_chanbw_words(float bw, radio_parms_t *radio_parms)
// ------------------------------------------------------------------------------------------------
{
    uint8_t e_index, m_index;
    bool index_found;
	if (radio_parms == NULL) {
		return;
	}
	index_found = false;
    radio_parms->chanbw_e = 3;
	radio_parms->chanbw_m = 3;
    for (e_index=0; e_index<4; e_index++) {
        for (m_index=0; m_index<4; m_index++) {
            if (bw > chanbw_limits[4*e_index + m_index]) {
                radio_parms->chanbw_e = e_index;
            	radio_parms->chanbw_m = m_index;
            	index_found = true;
            	break;
            }
        }
        if (index_found == true) {
        	break;
        }
    }
    return;
}

// ------------------------------------------------------------------------------------------------
// Calculate data rate, channel bandwidth and deviation words. Assumes 26 MHz crystal.
//   o DRATE = (Fxosc / 2^28) * (256 + DRATE_M) * 2^DRATE_E
//   o CHANBW = Fxosc / (8(4+CHANBW_M) * 2^CHANBW_E)
//   o DEVIATION = (Fxosc / 2^17) * (8 + DEVIATION_M) * 2^DEVIATION_E
void get_rate_words(rate_t data_rate, float mod_index, radio_parms_t *radio_parms)
// ------------------------------------------------------------------------------------------------
{
    float drate, deviat, f_xtal;
    if (mod_index == 0 || mod_index > 2.0){
    	return;
    }
	if (radio_parms == NULL){
		return;
	}
    drate = (float) rate_values[data_rate];

    deviat = 0.5 * drate * mod_index;
    f_xtal = (float) radio_parms->f_xtal;

    get_chanbw_words(2.0f*(deviat + drate), radio_parms); // Apply Carson's rule for bandwidth

    radio_parms->drate_e = (uint8_t) (floor(log2( drate*(1<<20) / f_xtal )));
    radio_parms->drate_m = (uint8_t) (((drate*(1<<28)) / (f_xtal * (1<<radio_parms->drate_e))) - 256);
    radio_parms->drate_e &= 0x0F; // it is 4 bits long

    radio_parms->deviat_e = (uint8_t) (floor(log2( deviat*(1<<14) / f_xtal )));
    radio_parms->deviat_m = (uint8_t) (((deviat*(1<<17)) / (f_xtal * (1<<radio_parms->deviat_e))) - 8);
    radio_parms->deviat_e &= 0x07; // it is 3 bits long
    radio_parms->deviat_m &= 0x07; // it is 3 bits long

    radio_parms->chanspc_e &= 0x03; // it is 2 bits long
}

void reset_parameters(void)
{
    radio_int_data.packet_receive = 0;
	radio_int_data.packet_send = 0;
    radio_int_data.mode = RADIOMODE_NONE;
}

void disable_lna_enable_pa(void)
{
	HAL_GPIO_WritePin(GPIOB, _LNA_EN_Pin, GPIO_PIN_SET);		// Disable LNA
	HAL_GPIO_WritePin(GPIOB, SW_CONTROL_Pin, GPIO_PIN_SET);		// Conmute to TX
	HAL_GPIO_WritePin(GPIOB, _5V_RF_EN_Pin, GPIO_PIN_RESET); 	// Enable POL 5v
	HAL_GPIO_WritePin(GPIOB, PA_EN_Pin, GPIO_PIN_SET);			// Enable PA
}

void disable_pa_enable_lna(void)
{
	HAL_GPIO_WritePin(GPIOB, PA_EN_Pin, GPIO_PIN_RESET);		// Disable PA
	HAL_GPIO_WritePin(GPIOB, _5V_RF_EN_Pin, GPIO_PIN_SET);		// Disable POL 5v
	HAL_GPIO_WritePin(GPIOB, SW_CONTROL_Pin, GPIO_PIN_RESET);	// Conmute to RX
	HAL_GPIO_WritePin(GPIOB, _LNA_EN_Pin, GPIO_PIN_RESET);		// Enable LNA
}

// ------------------------------------------------------------------------------------------------
// Inhibit operations by returning to IDLE state
void radio_turn_idle(spi_parms_t *spi_parms)
// ------------------------------------------------------------------------------------------------
{
	uint8_t state;
	reset_parameters();
	if (spi_parms == NULL){
		return;
	}
	CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &state);
	if (state == CC11xx_STATE_RXFIFO_OVERFLOW || state == CC11xx_STATE_TXFIFO_UNDERFLOW ){
		radio_flush_fifos(spi_parms);
	}
	/* If stays here too long, a WDT has to "jump" */
    CC_SPIStrobe(spi_parms, CC11xx_SIDLE);
    CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &state);
    while (state != CC11xx_STATE_IDLE){
    	CC_SPIStrobe(spi_parms, CC11xx_SIDLE);
    	CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &state);
    	if (state == CC11xx_STATE_RXFIFO_OVERFLOW || state == CC11xx_STATE_TXFIFO_UNDERFLOW ){
    		radio_flush_fifos(spi_parms);
    	}
    }
    /* If stays here too long, a WDT has to "jump" */
}

void radio_turn_rx_isr(spi_parms_t *spi_parms)
{
	uint8_t state;
	reset_parameters();

	if (spi_parms == NULL){
		return;
	}

	set_freq(0);
	/* Here change the Switch state */
	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_SYNC1, CC_SYNC1_UL);
	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_SYNC0, CC_SYNC0_UL);
	disable_pa_enable_lna();
    radio_int_data.mode = RADIOMODE_RX;
    CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &state);
    while (state != CC11xx_STATE_IDLE){
    	radio_turn_idle(spi_parms);
    	CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &state);
    }
    /* If stays here too long, a WDT has to "jump" */
    do{
		CC_SPIStrobe(spi_parms, CC11xx_SRX);
		CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &state);
		if (state == CC11xx_STATE_TXFIFO_UNDERFLOW || state == CC11xx_STATE_RXFIFO_OVERFLOW){
			radio_turn_idle(spi_parms);
		}
    }while (state != CC11xx_STATE_RX);
    /* If stays here too long, a WDT has to "jump" */
}

// ------------------------------------------------------------------------------------------------
// Allow Rx operations by returning to Rx state
void radio_turn_rx(spi_parms_t *spi_parms)
// ------------------------------------------------------------------------------------------------
{
	if (spi_parms == NULL){
		return;
	}
	radio_turn_rx_isr(spi_parms);
}

void radio_turn_tx(spi_parms_t *spi_parms)
{
	uint8_t state;
	if (spi_parms == NULL){
		return;
	}
	reset_parameters();

	set_freq(1);
	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_SYNC1, CC_SYNC1_DL);
	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_SYNC0, CC_SYNC0_DL);
	/* Here change the switch state */
	disable_lna_enable_pa();
	radio_int_data.mode = RADIOMODE_TX;
    do{
		CC_SPIStrobe(radio_int_data.spi_parms, CC11xx_STX);
		CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &state);
		if (state == CC11xx_STATE_TXFIFO_UNDERFLOW || state == CC11xx_STATE_RXFIFO_OVERFLOW){
			radio_turn_idle(spi_parms);
		}
    }while (state != CC11xx_STATE_TX);
    /* If stays here too long, a WDT has to "jump" */
}

void radio_calibrate(spi_parms_t *spi_parms)
{
	uint8_t reg;
	if (spi_parms == NULL){
		return;
	}
	radio_turn_idle(spi_parms);
	CC_SPIReadReg(spi_parms, CC11xx_FSCAL3, &reg);
	/* Read the register and set 5:4 bits to 1 */
	reg |= 0x30;
	CC_SPIWriteReg(spi_parms, CC11xx_FSCAL3,   0xEA);
	CC_SPIStrobe(spi_parms, CC11xx_SCAL);
	MDELAY(5);
	CC_SPIReadReg(spi_parms, CC11xx_FSCAL3, &reg);
	/* Read the register and set 5:4 bits to 0 */
	reg &= 0xCF;
	CC_SPIWriteReg(spi_parms, CC11xx_FSCAL3, reg);
	/* Go to RX...*/
	radio_turn_rx(spi_parms);
}

// ------------------------------------------------------------------------------------------------
// Flush Rx and Tx FIFOs
void radio_flush_fifos(spi_parms_t *spi_parms)
// ------------------------------------------------------------------------------------------------
{
	if (spi_parms == NULL) {
		return;
	}
    CC_SPIStrobe(spi_parms, CC11xx_SFRX); // Flush Rx FIFO
    CC_SPIStrobe(spi_parms, CC11xx_SFTX); // Flush Tx FIFO
}

uint8_t radio_csma(void)
{
	uint8_t carrier_sense;
	carrier_sense = CC11xx_GDO1();
	return carrier_sense;
}

// ------------------------------------------------------------------------------------------------
// Transmission of a block
static int radio_send_block(spi_parms_t *spi_parms, radio_parms_t *radio_parms)
// ------------------------------------------------------------------------------------------------
{
    uint16_t 	timeout;
    uint8_t 	channel_busy = 1;

	if (spi_parms == NULL) {
		return -1;
	}
	if (radio_parms == NULL) {
		return -1;
	}
    /* Set this shit to CCA --> Poll for this? Use ISR? */
    /* Lets start by polling GDO1 pin, if is 1, then Random Back off, look for 1 again and go! */
    /* The radio is always in RX */
#ifdef MAC_CSMA_ENABLE
    timeout = 0;
    while(channel_busy && timeout < radio_parms->timeout) {
		if(radio_csma() || radio_int_data.packet_receive) {
			/* 1 pkt time is ~ 250ms, thus if this happens sleeps from a random */
			channel_busy = 1;
			timeout += rand()%(radio_parms->timeout/4);
			MDELAY(timeout);
		}else{
			channel_busy = 1;
			timeout += rand()%(radio_parms->timeout/4);
			MDELAY(timeout);
			if(! (radio_csma() || radio_int_data.packet_receive) ) {
				channel_busy = 0;
			}
		}
    }
    /* So, we are not in TX state, we are not receiving a packet, lets sense the medium */
	if (timeout >= radio_parms->timeout && channel_busy)
	{
		/* Dropped packet */
		timeout = 0;
		radio_int_data.packet_not_tx_count++;
		return -1;
	}
#endif
	/* Wake up GDO */
	osSignalSet(CommsTaskHandle, COMMS_NOTIFY_SEND_REQ);
    return 0;
}

// ------------------------------------------------------------------------------------------------
// Transmission of a packet
int radio_send_packet(radio_packet_t *packet)
// ------------------------------------------------------------------------------------------------
{
	if (packet == NULL) {
		return -1;
	}

	/* Otherwise, something can be done! */
    radio_int_data.tx_count = CC11xx_PACKET_COUNT_SIZE; // same block size for all
    /* Append RS! */
    if (encode_rs_message(packet->raw, MAC_UNCODED_PACKET_SIZE, (uint8_t *) radio_int_data.tx_buf, CC11xx_PACKET_COUNT_SIZE) == CC11xx_PACKET_COUNT_SIZE){
        /* Timeout? */
    	return (radio_send_block(radio_int_data.spi_parms, radio_int_data.radio_parms));
    }else {
    	return -1;
    }
}

void enable_isr_routine(radio_parms_t *radio_parms)
{
	if (radio_parms == NULL){
		return;
	}
	radio_int_data.mode = RADIOMODE_NONE;
	radio_int_data.packet_rx_count = 0;
	radio_int_data.packet_tx_count = 0;
	radio_int_data.packet_not_tx_count = 0;
	radio_int_data.packet_rx_corrected = 0;
	radio_int_data.spi_parms = &spi_parms_it;
	radio_int_data.radio_parms = radio_parms;
	/* enable RX! */
	radio_turn_rx(radio_int_data.spi_parms);
	init_radio = true;
}


int  CC_SPIWriteReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t byte)
{
	if (spi_parms == NULL){
		return 1;
	}
    spi_parms->tx[0] = addr;
    spi_parms->tx[1] = byte;
    spi_parms->len = 2;

    disable_IT();
    spi_parms->ret = SPI_TRANSFER(spi_parms->tx, spi_parms->rx, spi_parms->len);
    enable_IT();

    if (spi_parms->ret != 0){
        return 1;
    }
    spi_parms->status = spi_parms->rx[0];
    return 0;
}

/* Correct write burst to be multiple calls to write reg */

int  CC_SPIWriteBurstReg(spi_parms_t *spi_parms, uint8_t addr, const uint8_t *bytes, uint8_t count)
{
    uint8_t i;
	if (spi_parms == NULL) {
		return 1;
	}
	if (bytes == NULL) {
		return 1;
	}
	if (count == 0) {
		return 1;
	}
    count %= 64;
    spi_parms->tx[0] = addr | CC11xx_WRITE_BURST;   // Send address

    for (i=1; i<count+1; i++)
    {
        spi_parms->tx[i] = bytes[i-1];
    }
    spi_parms->len = count+1;

    disable_IT();
    spi_parms->ret = SPI_TRANSFER(spi_parms->tx, spi_parms->rx, spi_parms->len);
    enable_IT();

    if (spi_parms->ret != 0){
        return 1;
    }
    spi_parms->status = spi_parms->rx[0];
    return 0;
}

int  CC_SPIReadReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t *byte)
{
	if (spi_parms == NULL){
		return 1;
	}
	if (byte == NULL){
		return 1;
	}

    spi_parms->tx[0] = addr | CC11xx_READ_SINGLE; // Send address
    spi_parms->tx[1] = 0; // Dummy write so we can read data
    spi_parms->len = 2;

    disable_IT();
    spi_parms->ret = SPI_TRANSFER(spi_parms->tx, spi_parms->rx, spi_parms->len);
    enable_IT();

    if (spi_parms->ret != 0){
        return 1;
    }
    *byte = spi_parms->rx[1];
    spi_parms->status = spi_parms->rx[0];
    return 0;
}

/* Correct read burst to be multiple calls to read reg */

int  CC_SPIReadBurstReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t *bytes, uint8_t count)
{    
    uint8_t i;
	if (spi_parms == NULL){
		return 1;
	}
	if (bytes == NULL){
		return 1;
	}
	if (count == 0){
		return 1;
	}
    count %= 64;
    spi_parms->tx[0] = addr | CC11xx_READ_BURST;   // Send address

    for (i=1; i<count+1; i++)
    {
        spi_parms->tx[i] = 0; // Dummy write so we can read data
    }

    spi_parms->len = count+1;

    disable_IT();
    spi_parms->ret = SPI_TRANSFER(spi_parms->tx, spi_parms->rx, spi_parms->len);
    enable_IT();

    if (spi_parms->ret != 0)
    {
        return 1;
    }

	memcpy(bytes, &spi_parms->rx[1], count);
    spi_parms->status = spi_parms->rx[0];
    return 0;
}

int  CC_SPIReadStatus(spi_parms_t *spi_parms, uint8_t addr, uint8_t *status)
{
	if (spi_parms == NULL){
		return 1;
	}
	if (status == NULL){
		return 1;
	}
    spi_parms->tx[0] = addr | CC11xx_READ_BURST;   // Send address
    spi_parms->tx[1] = 0; // Dummy write so we can read data
    spi_parms->len = 2;

    disable_IT();
    spi_parms->ret = SPI_TRANSFER(spi_parms->tx, spi_parms->rx, spi_parms->len);
    enable_IT();

    if (spi_parms->ret != 0)
    {
        return 1;
    }
    *status = spi_parms->rx[1];
    spi_parms->status = spi_parms->rx[0];
    return 0;
}

int  CC_SPIStrobe(spi_parms_t *spi_parms, uint8_t strobe)
{
	if (spi_parms == NULL){
		return 1;
	}
    spi_parms->tx[0] = strobe;   // Send strobe
    spi_parms->len = 1;

    disable_IT();
    spi_parms->ret = SPI_TRANSFER(spi_parms->tx, spi_parms->rx, spi_parms->len);
    enable_IT();

    if (spi_parms->ret != 0)
    {
        return 1;
    }
    spi_parms->status = spi_parms->rx[0];
    return 0;
}

int  CC_PowerupResetCCxxxx(spi_parms_t *spi_parms)
{
	if (spi_parms == NULL){
		return -1;
	}
	HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, CC1101_CS_Pin, GPIO_PIN_SET);
	/*
	 * PA5     ------> SPI1_SCK ----> to 1
	 * PA7     ------> SPI1_MOSI----> to 0
	 */
	HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, CC1101_CS_Pin, GPIO_PIN_RESET);
    spi_parms->tx[0] = CC11xx_SRES;   // Send strobe
    spi_parms->len = 1;
	MDELAY(1);
	while(HAL_GPIO_ReadPin(CC1101_GDO1_GPIO_Port, CC1101_GDO1_Pin) != GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spi_parms->tx, spi_parms->len, HAL_MAX_DELAY);
	while(HAL_GPIO_ReadPin(CC1101_GDO1_GPIO_Port, CC1101_GDO1_Pin) != GPIO_PIN_RESET);
    return 0;
}

inline void disable_IT(void)
{
	taskENTER_CRITICAL();
}

inline void enable_IT(void)
{
	taskEXIT_CRITICAL();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == CC1101_GDO0_Pin){
		/* Execute semaphore 1 */
		if (init_radio == true) {
			osSignalSet(CommsTaskHandle, GDO_NOTIFY_GDO0);
		}
	}
	if (GPIO_Pin == CC1101_GDO2_Pin){
		/* Execute semaphore 2 */
		if (init_radio == true) {
			osSignalSet(CommsTaskHandle, GDO_NOTIFY_GDO2);
		}
	}
}

static void change_frequency(void)
{
	comms_hk_data_t data;
	float temperature;
	ReturnHKData(&data);
	temperature = convert_temp_u16_f(data.housekeeping.int_temp);
	radio_int_data.radio_parms->f_off = (freq_temp_sensibility * temperature + freq_temp_offset);
}

void initialize_cc1101(void)
{
	set_freq_parameters(437.250e6f, 437.250e6f, 384e3f, 0.0f, &radio_parms);
	set_sync_parameters(PREAMBLE_4, SYNC_30_over_32, 500, &radio_parms);
	set_packet_parameters(false, true, &radio_parms);
	set_modulation_parameters(RADIO_MOD_GFSK, RATE_9600, 0.5f, &radio_parms);

	init_radio_config(&spi_parms_it, &radio_parms);
	enable_isr_routine(&radio_parms);
}

static simple_link_packet_t rx_packet_buffer;
static simple_link_packet_t tx_packet_buffer;
static chunk_handler_t 		hchunk_tx, hchunk_rx;
static radio_packet_t 		tx_radio_packet;

static uint8_t test_out[223];
static uint8_t test_in[255];

void cc1101_work(void)
{
	link_layer_packet_t *ll_packet;
	int32_t signals_to_wait;
	osEvent signal_received;
	uint8_t initial_tx_count;
	int i;
	init_chunk_handler(&hchunk_rx);
	for (i = 0; i < 255; i++) {
		test_in[i] = rand()%0xFF;
	}
	decode_rs_message(test_in, 255, test_out, 223);

	/* Init done */
	while(1) {
		signals_to_wait = GDO_NOTIFY_GDO0 | GDO_NOTIFY_GDO2 | COMMS_NOTIFY_SEND_REQ;
		signal_received = osSignalWait(signals_to_wait, 1000);
		if (signal_received.status == osEventSignal) {
			if (signal_received.value.signals & GDO_NOTIFY_GDO0) {
				if (gdo0_isr() == func_data) {
					if(set_new_packet_to_chunk(&hchunk_rx, &rx_radio_packet, rx_packet_buffer.fields.payload) > 0) {
						ll_packet = (link_layer_packet_t *) rx_packet_buffer.fields.payload;
						/* Send that towards UART */
						if (set_simple_link_packet(ll_packet, ll_packet->fields.len + LINK_LAYER_HEADER_SIZE, 0, 0, &rx_packet_buffer) > 0) {
							xQueueSend(LinkLayerRxQueueHandle, &rx_packet_buffer, 0);
						}
					}
				}
			}
			if (signal_received.value.signals & GDO_NOTIFY_GDO2) {
				gdo2_isr();
			}
			if (signal_received.value.signals & COMMS_NOTIFY_SEND_REQ) {
				radio_turn_idle(radio_int_data.spi_parms);
				change_frequency();
				// Initial number of bytes to put in FIFO is either the number of bytes to send or the FIFO size whichever is
				// the smallest. Actual size blocks you need to take size minus one byte.
				initial_tx_count = (radio_int_data.tx_count > CC11xx_FIFO_SIZE-1 ? CC11xx_FIFO_SIZE-1 : radio_int_data.tx_count);
				// Initial fill of TX FIFO
				CC_SPIWriteBurstReg(radio_int_data.spi_parms, CC11xx_TXFIFO, (uint8_t *) radio_int_data.tx_buf, initial_tx_count);
				radio_int_data.byte_index = initial_tx_count;
				radio_int_data.bytes_remaining = radio_int_data.tx_count - initial_tx_count;
				radio_turn_tx(radio_int_data.spi_parms);
			}
		}else {
			/* get rssi */
			/* this gets rssi! */
			if (radio_int_data.packet_receive == 0 && radio_int_data.mode == RADIOMODE_RX) {
				actual_rssi = get_dec_rssi();
				change_frequency();
				set_freq(0);
			}
		}
	}
}

void csma_tx_work(void)
{
	link_layer_packet_t *ll_packet;
	init_chunk_handler(&hchunk_tx);
	/* Init done */
	while(1) {
		/* Wait for notification here */
		if (xQueueReceive(RadioPacketTxQueueHandle, &tx_packet_buffer, osWaitForever) == pdTRUE) {
			ll_packet = (link_layer_packet_t *) tx_packet_buffer.fields.payload;
			while (get_new_packet_from_chunk(&hchunk_tx, ll_packet->raw, ll_packet->fields.len + LINK_LAYER_HEADER_SIZE,
												tx_packet_buffer.fields.config2, &tx_radio_packet) > 0)
			{
				if (radio_send_packet(&tx_radio_packet) == 0) {
					osSignalWait(COMMS_NOTIFY_END_TX, osWaitForever);
				}
				/* wait for end of packet */
			}
			if (radio_send_packet(&tx_radio_packet) == 0) {
				osSignalWait(COMMS_NOTIFY_END_TX, osWaitForever);
			}
			/* wait for end of packet */
		}
	}
}

