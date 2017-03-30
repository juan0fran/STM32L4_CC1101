#include "cc1101_routine.h"

#define TX_FIFO_REFILL 58 // With the default FIFO thresholds selected this is the number of bytes to refill the Tx FIFO
#define RX_FIFO_UNLOAD 59 // With the default FIFO thresholds selected this is the number of bytes to unload from the Rx FIFO

int __errno;

circ_buff_t circular_cc1101_queue;

static spi_parms_t spi_parms_it;
radio_int_data_t radio_int_data;
static bool init_radio = false;

static radio_packet_t packet;

static float chanbw_limits[] = {
    812000.0, 650000.0, 541000.0, 464000.0, 406000.0, 325000.0, 270000.0, 232000.0,
    203000.0, 162000.0, 135000.0, 116000.0, 102000.0, 81000.0, 68000.0, 58000.0
};

static uint32_t rate_values[] = {
    50, 110, 300, 600, 1200, 2400, 4800, 9600,
    14400, 19200, 28800, 38400, 57600, 76800, 115200,
};

static bool 	force_isr_disable = false;

static int 		CC_SPIWriteReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t byte);
static int     	CC_SPIWriteBurstReg(spi_parms_t *spi_parms, uint8_t addr, const uint8_t *bytes, uint8_t count);
static int     	CC_SPIReadReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t *byte);
static int     	CC_SPIReadBurstReg(spi_parms_t *spi_parms, uint8_t addr, uint8_t *bytes, uint8_t count);
static int     	CC_SPIReadStatus(spi_parms_t *spi_parms, uint8_t addr, uint8_t *status);
static int     	CC_SPIStrobe(spi_parms_t *spi_parms, uint8_t strobe);
static int     	CC_PowerupResetCCxxxx(spi_parms_t *spi_parms);

static void    	disable_IT(void);
static void    	enable_IT(void);

static uint32_t get_freq_word(uint32_t freq_xtal, uint32_t freq_hz);
static uint32_t get_if_word(uint32_t freq_xtal, uint32_t if_hz);
static uint8_t 	get_offset_word(uint32_t freq_xtal, uint32_t offset_hz);

static void     get_chanbw_words(float bw, radio_parms_t *radio_parms);
static void     get_rate_words(rate_t data_rate, float mod_index, radio_parms_t *radio_parms);

static void     radio_turn_idle(spi_parms_t *spi_parms);
static void     radio_turn_rx_isr(spi_parms_t *spi_parms);
static void     radio_turn_rx(spi_parms_t *spi_parms);
static void		radio_turn_tx(spi_parms_t *spi_parms);

static uint8_t 	radio_csma(void);

static void     radio_flush_fifos(spi_parms_t *spi_parms);

static void 	reset_parameters(void);

uint16_t error_cnt = 0;
uint16_t spi_error_cnt = 0;
void cc1101_check(void)
{
	uint8_t reg_word;
    if (init_radio == false){
        return;
    }
	/* this is in charge of checking cc1101 is OK or not */
	/* time between calls is 1 second */

    if (CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_MARCSTATE, &reg_word) == 0){
    	if (reg_word == CC11xx_STATE_STARTCAL){
    		return;
    	}
    	if ( (radio_int_data.mode == RADIOMODE_NONE) && (reg_word == CC11xx_STATE_IDLE) ){

    	}else if ( (radio_int_data.mode == RADIOMODE_RX) && (reg_word == CC11xx_STATE_RX) ){

    	}else if ( (radio_int_data.mode == RADIOMODE_TX) && (reg_word == CC11xx_STATE_TX) ){

    	}else{
    		error_cnt++;
    	}
    }else{
    	spi_error_cnt++;
    }
    if ( (radio_int_data.mode == RADIOMODE_RX) && (reg_word == CC11xx_STATE_RX) ){
    	if (!radio_int_data.packet_receive){
    		radio_turn_idle(radio_int_data.spi_parms);
    		radio_turn_rx_isr(radio_int_data.spi_parms);
    	}
    }
}

// ------------------------------------------------------------------------------------------------
// Processes packets up to 255 bytes
void gdo0_isr(void)
// ------------------------------------------------------------------------------------------------
{
    uint8_t int_line, status;
    if (init_radio == false){
        return;
    }
    int_line = CC11xx_GDO0(); // Sense interrupt line to determine if it was a raising or falling edge

    if (radio_int_data.mode == RADIOMODE_RX){
        if (int_line){         
        	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_IOCFG2, 0x00);
            radio_int_data.byte_index = 0;
            radio_int_data.rx_count = CC11xx_PACKET_COUNT_SIZE;
            radio_int_data.bytes_remaining = radio_int_data.rx_count;
            radio_int_data.packet_receive = 1; // reception is in progress
        }else{
            if (radio_int_data.packet_receive){

                CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_RXBYTES, &status);
                reset_parameters(); // reception is done
                if ( (status&0x80) == 0x80){ /* Overflow */
                    radio_turn_idle(radio_int_data.spi_parms);
                }else{
                    CC_SPIReadBurstReg(radio_int_data.spi_parms, CC11xx_RXFIFO, (uint8_t *) &(radio_int_data.rx_buf[radio_int_data.byte_index]), radio_int_data.bytes_remaining);
                    radio_int_data.byte_index += radio_int_data.bytes_remaining;
                    radio_int_data.bytes_remaining = 0;

                    /* get lqi and rssi */
                    CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_LQI, &status);
                    packet.fields.lqi = status&0x7F;

                    CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_RSSI, &status);
                    packet.fields.rssi = status;

                    if (decode_rs_message((uint8_t *) radio_int_data.rx_buf, CC11xx_PACKET_COUNT_SIZE, packet.raw, MAC_UNCODED_PACKET_SIZE) == MAC_UNCODED_PACKET_SIZE){
                    	enqueue(&circular_cc1101_queue, &packet);
                    	radio_int_data.packet_rx_count++;
                    }
			    }
                radio_turn_rx_isr(radio_int_data.spi_parms);
            }
        }    
    }else if (radio_int_data.mode == RADIOMODE_TX){
        if (int_line){
        	CC_SPIWriteReg(radio_int_data.spi_parms, CC11xx_IOCFG2, 0x02); // GDO2 output pin config TX mode
            radio_int_data.packet_send = 1; // Assert packet transmission after sync has been sent
        }else{
            if (radio_int_data.packet_send){
                CC_SPIReadStatus(radio_int_data.spi_parms, CC11xx_TXBYTES, &status);
                if ( (status&0x80) == 0x80){ /* Underflow */
                	reset_parameters();
                    radio_turn_idle(radio_int_data.spi_parms);
                }else{
                	reset_parameters(); // De-assert packet transmission after packet has been sent
                    radio_int_data.packet_tx_count++;
                    if ((radio_int_data.bytes_remaining)){
                        radio_turn_idle(radio_int_data.spi_parms);          
                    }
                }
            }
            radio_turn_rx_isr(radio_int_data.spi_parms);
        }
    }
}

// ------------------------------------------------------------------------------------------------
// Processes packets that do not fit in Rx or Tx FIFOs and 255 bytes long maximum
// FIFO threshold interrupt handler 
void gdo2_isr(void)
// ------------------------------------------------------------------------------------------------
{
    uint8_t int_line, bytes_to_send;
    if (init_radio == false){
        return;
    }
    int_line = CC11xx_GDO2(); // Sense interrupt line to determine if it was a raising or falling edge

    if ((radio_int_data.mode == RADIOMODE_RX) && (int_line)){
        if (radio_int_data.packet_receive){
            /* if this shit wants to write but rx_buf will overload, just break */
            CC_SPIReadBurstReg(radio_int_data.spi_parms, CC11xx_RXFIFO, (uint8_t *) &(radio_int_data.rx_buf[radio_int_data.byte_index]), RX_FIFO_UNLOAD);
            /* Check for status byte in each */
            radio_int_data.byte_index += RX_FIFO_UNLOAD;
            radio_int_data.bytes_remaining -= RX_FIFO_UNLOAD;    
            return;        
        }
    }
    if ((radio_int_data.mode == RADIOMODE_TX) && (!int_line)){
        if ((radio_int_data.packet_send) && (radio_int_data.bytes_remaining > 0)){
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

int set_freq_parameters(float freq_hz, float freq_if, float freq_off, radio_parms_t * radio_parms)
{
    radio_parms->freq_hz        = freq_hz;
    radio_parms->f_if           = freq_if;
	radio_parms->f_off 			= freq_off;

    /* Set the nominal parameters */
    radio_parms->f_xtal        = 26000000;
    radio_parms->chanspc_m     = 0;                // Do not use channel spacing for the moment defaulting to 0
    radio_parms->chanspc_e     = 0;                // Do not use channel spacing for the moment defaulting to 0
		
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
    // Write register settings
	if (spi_parms == NULL){
		return -1;
	}
	if (radio_parms == NULL){
		return -1;
	}
	force_isr_disable = true;
    if (CC_PowerupResetCCxxxx(spi_parms) != 0){
    	return -1;
    }

    /* Patable Write here? */
    /* First read it */
    uint8_t patable[8];
    CC_SPIReadBurstReg(spi_parms, CC11xx_PATABLE, patable, sizeof(patable));
    patable[0] = 0xC0;
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

    force_isr_disable = false;

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
		reg_word = get_offset_word(radio_parms->f_xtal, radio_parms->f_off);
    CC_SPIWriteReg(spi_parms, CC11xx_FSCTRL0,  reg_word); // Freq synthesizer control.

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
    radio_parms->freq_word = get_freq_word(radio_parms->f_xtal, radio_parms->freq_hz);
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
    CC_SPIWriteReg(spi_parms, CC11xx_AGCCTRL2, 0x07); // AGC control.

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
    CC_SPIWriteReg(spi_parms, CC11xx_AGCCTRL1, 0x00); // AGC control.

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
    //   2 (10): Manually freeze the analogue gain setting and continue to adjust the digital gain. 
    //   3 (11): Manually freezes both the analogue and the digital gain setting. Used for manually overriding the gain.
    // o bits 0:1: FILTER_LENGTH: 
    //   2-FSK, 4-FSK, MSK: Sets the averaging length for the amplitude from the channel filter.    |  
    //   ASK ,OOK: Sets the OOK/ASK decision boundary for OOK/ASK reception.
    //   Value : #samples: OOK/ASK decixion boundary
    //   0 (00):        8: 4 dB
    //   1 (01):       16: 8 dB
    //   2 (10):       32: 12 dB
    //   3 (11):       64: 16 dB  
    CC_SPIWriteReg(spi_parms, CC11xx_AGCCTRL0, 0xB0); // AGC control.

    // FREND1: Front End RX Configuration
    // o bits 7:6: LNA_CURRENT: Adjusts front-end LNA PTAT current output
    // o bits 5:4: LNA2MIX_CURRENT: Adjusts front-end PTAT outputs
    // o bits 3:2: LODIV_BUF_CURRENT_RX: Adjusts current in RX LO buffer (LO input to mixer)
    // o bits 1:0: MIX_CURRENT: Adjusts current in mixer
    CC_SPIWriteReg(spi_parms, CC11xx_FREND1,   0xB6); // Front end RX configuration.

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
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL3,   0xEA); // Frequency synthesizer cal.

    // FSCAL2: Frequency Synthesizer Calibration
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL2,   0x0A); // Frequency synthesizer cal.
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL1,   0x00); // Frequency synthesizer cal.
    CC_SPIWriteReg(spi_parms, CC11xx_FSCAL0,   0x11); // Frequency synthesizer cal.

    // TEST2: Various test settings. The value to write in this field is given by the SmartRF Studio software.
    CC_SPIWriteReg(spi_parms, CC11xx_TEST2,    0x88); // Various test settings.

    // TEST1: Various test settings. The value to write in this field is given by the SmartRF Studio software.
    CC_SPIWriteReg(spi_parms, CC11xx_TEST1,    0x31); // Various test settings.

    // TEST0: Various test settings. The value to write in this field is given by the SmartRF Studio software.
    CC_SPIWriteReg(spi_parms, CC11xx_TEST0,    0x09); // Various test settings.

    return 0;
}

int set_freq(spi_parms_t * spi_parms, radio_parms_t * radio_parms)
{
	uint8_t reg_word;
	if (spi_parms == NULL){
		return 0;
	}
	if (radio_parms == NULL){
		return 0;
	}

	reg_word = get_offset_word(radio_parms->f_xtal, radio_parms->f_off);
    CC_SPIWriteReg(spi_parms, CC11xx_FSCTRL0,  reg_word); // Freq synthesizer control.

    radio_parms->if_word = get_if_word(radio_parms->f_xtal, radio_parms->f_if);
    CC_SPIWriteReg(spi_parms, CC11xx_FSCTRL1, (radio_parms->if_word & 0x1F)); // Freq synthesizer control.

    radio_parms->freq_word = get_freq_word(radio_parms->f_xtal, radio_parms->freq_hz);
    CC_SPIWriteReg(spi_parms, CC11xx_FREQ2,    ((radio_parms->freq_word>>16) & 0xFF)); // Freq control word, high byte
    CC_SPIWriteReg(spi_parms, CC11xx_FREQ1,    ((radio_parms->freq_word>>8)  & 0xFF)); // Freq control word, mid byte.
    CC_SPIWriteReg(spi_parms, CC11xx_FREQ0,    (radio_parms->freq_word & 0xFF));       // Freq control word, low byte.
    return 0;
}

int set_mod(spi_parms_t * spi_parms, radio_parms_t * radio_parms)
{
	uint8_t reg_word;

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

    reg_word = (radio_parms->deviat_e<<4) + (radio_parms->deviat_m);
    CC_SPIWriteReg(spi_parms, CC11xx_DEVIATN,  reg_word); // Modem dev (when FSK mod en)
    return 0;
}

// ------------------------------------------------------------------------------------------------
// Calculate RSSI in dBm from decimal RSSI read out of RSSI status register
float rssi_dbm(uint8_t rssi_dec)
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

float lqi_status(uint8_t lqi)
{
	return (1.0 - ((float) lqi/127.0)) * 100.0;
}

// ------------------------------------------------------------------------------------------------
// Calculate frequency word FREQ[23..0]
uint32_t get_freq_word(uint32_t freq_xtal, uint32_t freq_hz)
// ------------------------------------------------------------------------------------------------
{
    uint64_t res; // calculate on 64 bits to save precision
    res = ((uint64_t) freq_hz * (uint64_t) (1<<16)) / ((uint64_t) freq_xtal);
    return (uint32_t) res;
}

// ------------------------------------------------------------------------------------------------
// Calculate frequency word FREQ[23..0]
uint32_t get_if_word(uint32_t freq_xtal, uint32_t if_hz)
// ------------------------------------------------------------------------------------------------
{
    return (if_hz * (1<<10)) / freq_xtal;
}

uint8_t get_offset_word(uint32_t freq_xtal, uint32_t offset_hz)
{
		return ((offset_hz * (1 << 14)) / freq_xtal) &0xFF;
}

// ------------------------------------------------------------------------------------------------
// Calculate CHANBW words according to CC1101 bandwidth steps
void get_chanbw_words(float bw, radio_parms_t *radio_parms)
// ------------------------------------------------------------------------------------------------
{
    uint8_t e_index, m_index;
	if (radio_parms == NULL){
		return;
	}
    for (e_index=0; e_index<4; e_index++)
    {
        for (m_index=0; m_index<4; m_index++)
        {
            if (bw > chanbw_limits[4*e_index + m_index])
            {
                radio_parms->chanbw_e = e_index;
                radio_parms->chanbw_m = m_index;
                return;
            }
        }
    }
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

    deviat = drate * mod_index;
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

	uint32_t freq_word;
	if (spi_parms == NULL){
		return;
	}


	freq_word = get_freq_word(26e6, 434.92e6);
	CC_SPIWriteReg(spi_parms, CC11xx_FREQ2,    ((freq_word>>16) & 0xFF)); // Freq control word, high byte
	CC_SPIWriteReg(spi_parms, CC11xx_FREQ1,    ((freq_word>>8)  & 0xFF)); // Freq control word, mid byte.
	CC_SPIWriteReg(spi_parms, CC11xx_FREQ0,    (freq_word & 0xFF));       // Freq control word, low byte.


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

	uint32_t freq_word;

	freq_word = get_freq_word(26e6, 433.92e6);
	CC_SPIWriteReg(spi_parms, CC11xx_FREQ2,    ((freq_word>>16) & 0xFF)); // Freq control word, high byte
	CC_SPIWriteReg(spi_parms, CC11xx_FREQ1,    ((freq_word>>8)  & 0xFF)); // Freq control word, mid byte.
	CC_SPIWriteReg(spi_parms, CC11xx_FREQ0,    (freq_word & 0xFF));       // Freq control word, low byte.

	radio_int_data.mode = RADIOMODE_TX;
    do{
		CC_SPIStrobe(spi_parms, CC11xx_STX);
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
	if (spi_parms == NULL){
		return;
	}
    CC_SPIStrobe(spi_parms, CC11xx_SFRX); // Flush Rx FIFO
    CC_SPIStrobe(spi_parms, CC11xx_SFTX); // Flush Tx FIFO
}

uint8_t radio_csma(void)
{
	return CC11xx_GDO1();
}

// ------------------------------------------------------------------------------------------------
// Transmission of a block
static void radio_send_block(spi_parms_t *spi_parms, radio_parms_t *radio_parms)
// ------------------------------------------------------------------------------------------------
{
    uint8_t  	initial_tx_count; // Number of bytes to send in first batch
    volatile uint16_t 	timeout;
    uint8_t 	channel_busy = 1;

	if (spi_parms == NULL){
		return;
	}
	if (radio_parms == NULL){
		return;
	}
    /* Set this shit to CCA --> Poll for this? Use ISR? */
    /* Lets start by polling GDO2 pin, if is 1, then Random Back off, look for 1 again and go! */
    /* The radio is always in RX */
#if 1
    timeout = 0;
    while(channel_busy && timeout < radio_parms->timeout){
		if(radio_csma() || radio_int_data.packet_receive){
			/* 1 pkt time is ~ 250ms, thus if this happens sleeps from a random */
			channel_busy = 1;
			timeout += rand()%(radio_parms->timeout/4);
			MDELAY(timeout);
		}else{
			channel_busy = 1;
			timeout += rand()%(radio_parms->timeout/4);
			MDELAY(timeout);
			if(! (radio_csma() || radio_int_data.packet_receive) ){
				channel_busy = 0;
			}
		}
    }
    /* So, we are not in TX state, we are not receiving a packet, lets sense the medium */
	if (timeout >= radio_parms->timeout && channel_busy)
	{
		/* Dropped packet */
		timeout = 0;
		return;
	}
#endif
	radio_turn_idle(spi_parms);

    // Initial number of bytes to put in FIFO is either the number of bytes to send or the FIFO size whichever is
    // the smallest. Actual size blocks you need to take size minus one byte.
    initial_tx_count = (radio_int_data.tx_count > CC11xx_FIFO_SIZE-1 ? CC11xx_FIFO_SIZE-1 : radio_int_data.tx_count);
    // Initial fill of TX FIFO
    CC_SPIWriteBurstReg(spi_parms, CC11xx_TXFIFO, (uint8_t *) radio_int_data.tx_buf, initial_tx_count);
    radio_int_data.byte_index = initial_tx_count;
    radio_int_data.bytes_remaining = radio_int_data.tx_count - initial_tx_count;

    radio_turn_tx(spi_parms);
}

// ------------------------------------------------------------------------------------------------
// Transmission of a packet
void radio_send_packet(spi_parms_t *spi_parms, radio_parms_t * radio_parms, radio_packet_t * packet)
// ------------------------------------------------------------------------------------------------
{
	if (spi_parms == NULL){
		return;
	}
	if (radio_parms == NULL){
		return;
	}
	if (packet == NULL){
		return;
	}

	/* Otherwise, something can be done! */
    radio_int_data.tx_count = CC11xx_PACKET_COUNT_SIZE; // same block size for all
    /* Append RS! */
    /* We have to wait to this to finish before copying to the buffer ! */
    /* Stupid motherfucker */
	while(radio_int_data.mode == RADIOMODE_TX){
		MDELAY(5);
	}
    if (encode_rs_message(packet->raw, MAC_UNCODED_PACKET_SIZE, (uint8_t *) radio_int_data.tx_buf, CC11xx_PACKET_COUNT_SIZE) == CC11xx_PACKET_COUNT_SIZE){
        /* Timeout? */
    	radio_send_block(spi_parms, radio_parms);
    }
}

void enable_isr_routine(spi_parms_t * spi_parms, radio_parms_t * radio_parms)
{
	if (spi_parms == NULL){
		return;
	}
	if (radio_parms == NULL){
		return;
	}
	radio_int_data.mode = RADIOMODE_NONE;
	radio_int_data.packet_rx_count = 0;
	radio_int_data.packet_tx_count = 0;
	radio_int_data.spi_parms = &spi_parms_it;
	radio_int_data.radio_parms = radio_parms;
	/* enable RX! */
	queue_init(&circular_cc1101_queue, sizeof(radio_packet_t), 20);
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
	uint8_t reg_word;
	if (spi_parms == NULL){
		return -1;
	}
	/* Read if spi is working && cc is alive */
    if (CC_SPIReadStatus(spi_parms, CC11xx_MARCSTATE, &reg_word) != 0){
    	return -1;
    }
    do{
    	CC_SPIStrobe(spi_parms, CC11xx_SRES);
    	MDELAY(10);
    	CC_SPIReadStatus(spi_parms, CC11xx_MARCSTATE, &reg_word);
    }while(reg_word != CC11xx_STATE_IDLE);
    return 0;
}


void disable_IT(void)
{
    /* Must be changed */
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void enable_IT(void)
{
	/* Must be changed */
	if (!force_isr_disable){
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Do stuff */
	cc1101_check();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == CC1101_GDO0_Pin){
		gdo0_isr();
		return;
	}
	if (GPIO_Pin == CC1101_GDO2_Pin){
		gdo2_isr();
		return;
	}
}
