/***************************************************************************************************
*  File:        cc1101_routine.h                                                                   *
*  Authors:     Joan Francesc Muñoz Martin <JFM>                                                   *
*                                                                                                  *
*  Creation:    27-11-2017                                                                         *
*  Description: CC1101 definition file, includes packet definition, cc1101 register defitnion...   *
*                                                                                                  *
*  This file is part of a project developed by Nano-Satellite and Payload Laboratory (NanoSat Lab) *
*  at Technical University of Catalonia - UPC BarcelonaTech.                                       *
*                                                                                                  *
* ------------------------------------------------------------------------------------------------ *
*  Changelog:                                                                                      *
*  v#   Date            Author  Description                                                        *
*  0.1  27-11-2017      <JFM>   <First version>                                                    *
***************************************************************************************************/

#ifndef __CC1101_DEFINES_H__
#define __CC1101_DEFINES_H__

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "cc1101_wrapper.h"

#include "command_parser.h"
#include "link_layer.h"
#include "rs_work.h"

#include "housekeeping.h"

#define COMMS_WAIT_TIME                 1000 /* ms */

/* Preamble amount */
typedef enum preamble_e {
    PREAMBLE_2,
    PREAMBLE_3,
    PREAMBLE_4,
    PREAMBLE_6,
    PREAMBLE_8,
    PREAMBLE_12,
    PREAMBLE_16,
    PREAMBLE_24,
    NUM_PREAMBLE
} preamble_t;

/* Sync word type */
typedef enum sync_word_e
{
    NO_SYNC = 0,              // No preamble/sync
    SYNC_15_OVER_16,          // 15/16 sync word bits detected
    SYNC_16_OVER_16,          // 16/16 sync word bits detected
    SYNC_30_over_32,          // 30/32 sync word bits detected
    SYNC_CARRIER,             // No preamble/sync, carrier-sense above threshold
    SYNC_15_OVER_16_CARRIER,  // 15/16 + carrier-sense above threshold
    SYNC_16_OVER_16_CARRIER,  // 16/16 + carrier-sense above threshold
    SYNC_30_over_32_CARRIER   // 30/32 + carrier-sense above threshold
} sync_word_t;

/* Different modulation types */
typedef enum radio_modulation_e {
    RADIO_MOD_FSK2  = 0,
    RADIO_MOD_GFSK  = 1,
    RADIO_MOD_OOK   = 3,
    RADIO_MOD_FSK4  = 4,
    RADIO_MOD_MSK   = 7,
    NUM_RADIO_MOD
} radio_modulation_t;

/* Available data rates */
typedef enum rate_e {
    RATE_50,
    RATE_110,
    RATE_300,
    RATE_600,
    RATE_1200,
    RATE_2400,
    RATE_4800,
    RATE_9600,
    RATE_14400,
    RATE_19200,
    RATE_28800,
    RATE_38400,
    RATE_57600,
    RATE_76800,
    RATE_115200,
    NUM_RATE
} rate_t;

/* Radio mode */
typedef enum radio_mode_e
{
    RADIOMODE_NONE = 0,
    RADIOMODE_RX,
    RADIOMODE_TX,
    NUM_RADIOMODE
} radio_mode_t;

typedef enum CC11xx_state_e {
    CC11xx_STATE_SLEEP = 0,
    CC11xx_STATE_IDLE,
    CC11xx_STATE_XOFF,
    CC11xx_STATE_VCOON_MC,
    CC11xx_STATE_REGON_MC,
    CC11xx_STATE_MANCAL,
    CC11xx_STATE_VCOON,
    CC11xx_STATE_REGON,
    CC11xx_STATE_STARTCAL,
    CC11xx_STATE_BWBOOST,
    CC11xx_STATE_FS_LOCK,
    CC11xx_STATE_IFADCON,
    CC11xx_STATE_ENDCAL,
    CC11xx_STATE_RX,
    CC11xx_STATE_RX_END,
    CC11xx_STATE_RX_RST,
    CC11xx_STATE_TXRX_SWITCH,
    CC11xx_STATE_RXFIFO_OVERFLOW,
    CC11xx_STATE_FSTXON,
    CC11xx_STATE_TX,
    CC11xx_STATE_TX_END,
    CC11xx_STATE_RXTX_SWITCH,
    CC11xx_STATE_TXFIFO_UNDERFLOW
} CC11xx_state_t;

// Configuration Registers
#define CC11xx_IOCFG2       0x00        // GDO2 output pin configuration
#define CC11xx_IOCFG1       0x01        // GDO1 output pin configuration
#define CC11xx_IOCFG0       0x02        // GDO0 output pin configuration
#define CC11xx_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC11xx_SYNC1        0x04        // Sync word, high byte
#define CC11xx_SYNC0        0x05        // Sync word, low byte
#define CC11xx_PKTLEN       0x06        // Packet length
#define CC11xx_PKTCTRL1     0x07        // Packet automation control
#define CC11xx_PKTCTRL0     0x08        // Packet automation control
#define CC11xx_ADDR         0x09        // Device address
#define CC11xx_CHANNR       0x0A        // Channel number
#define CC11xx_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC11xx_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC11xx_FREQ2        0x0D        // Frequency control word, high byte
#define CC11xx_FREQ1        0x0E        // Frequency control word, middle byte
#define CC11xx_FREQ0        0x0F        // Frequency control word, low byte
#define CC11xx_MDMCFG4      0x10        // Modem configuration
#define CC11xx_MDMCFG3      0x11        // Modem configuration
#define CC11xx_MDMCFG2      0x12        // Modem configuration
#define CC11xx_MDMCFG1      0x13        // Modem configuration
#define CC11xx_MDMCFG0      0x14        // Modem configuration
#define CC11xx_DEVIATN      0x15        // Modem deviation setting
#define CC11xx_MCSM2        0x16        // Main Radio Cntrl State Machine config
#define CC11xx_MCSM1        0x17        // Main Radio Cntrl State Machine config
#define CC11xx_MCSM0        0x18        // Main Radio Cntrl State Machine config
#define CC11xx_FOCCFG       0x19        // Frequency Offset Compensation config
#define CC11xx_BSCFG        0x1A        // Bit Synchronization configuration
#define CC11xx_AGCCTRL2     0x1B        // AGC control
#define CC11xx_AGCCTRL1     0x1C        // AGC control
#define CC11xx_AGCCTRL0     0x1D        // AGC control
#define CC11xx_WOREVT1      0x1E        // High byte Event 0 timeout
#define CC11xx_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CC11xx_WORCTRL      0x20        // Wake On Radio control
#define CC11xx_FREND1       0x21        // Front end RX configuration
#define CC11xx_FREND0       0x22        // Front end TX configuration
#define CC11xx_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC11xx_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC11xx_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC11xx_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC11xx_RCCTRL1      0x27        // RC oscillator configuration
#define CC11xx_RCCTRL0      0x28        // RC oscillator configuration
#define CC11xx_FSTEST       0x29        // Frequency synthesizer cal control
#define CC11xx_PTEST        0x2A        // Production test
#define CC11xx_AGCTEST      0x2B        // AGC test
#define CC11xx_TEST2        0x2C        // Various test settings
#define CC11xx_TEST1        0x2D        // Various test settings
#define CC11xx_TEST0        0x2E        // Various test settings

// Strobe commands
#define CC11xx_SRES         0x30        // Reset chip.
#define CC11xx_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define CC11xx_SXOFF        0x32        // Turn off crystal oscillator.
#define CC11xx_SCAL         0x33        // Calibrate freq synthesizer & disable
#define CC11xx_SRX          0x34        // Enable RX.
#define CC11xx_STX          0x35        // Enable TX.
#define CC11xx_SIDLE        0x36        // Exit RX / TX
#define CC11xx_SAFC         0x37        // AFC adjustment of freq synthesizer
#define CC11xx_SWOR         0x38        // Start automatic RX polling sequence
#define CC11xx_SPWD         0x39        // Enter pwr down mode when CSn goes hi
#define CC11xx_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC11xx_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC11xx_SWORRST      0x3C        // Reset real time clock.
#define CC11xx_SNOP         0x3D        // No operation.

// Status registers
#define CC11xx_PARTNUM      0x30        // Part number
#define CC11xx_VERSION      0x31        // Current version number
#define CC11xx_FREQEST      0x32        // Frequency offset estimate
#define CC11xx_LQI          0x33        // Demodulator estimate for link quality
#define CC11xx_RSSI         0x34        // Received signal strength indication
#define CC11xx_MARCSTATE    0x35        // Control state machine state
#define CC11xx_WORTIME1     0x36        // High byte of WOR timer
#define CC11xx_WORTIME0     0x37        // Low byte of WOR timer
#define CC11xx_PKTSTATUS    0x38        // Current GDOx status and packet status
#define CC11xx_VCO_VC_DAC   0x39        // Current setting from PLL cal module
#define CC11xx_TXBYTES      0x3A        // Underflow and # of bytes in TXFIFO
#define CC11xx_RXBYTES      0x3B        // Overflow and # of bytes in RXFIFO
#define CC11xx_NUM_RXBYTES  0x7F        // Mask "# of bytes" field in _RXBYTES

// Other memory locations
#define CC11xx_PATABLE      0x3E
#define CC11xx_TXFIFO       0x3F
#define CC11xx_RXFIFO       0x3F

// Masks for appended status bytes
#define CC11xx_LQI_RX       0x01        // Position of LQI byte
#define CC11xx_CRC_OK       0x80        // Mask "CRC_OK" bit within LQI byte

// Definitions to support burst/single access:
#define CC11xx_WRITE_BURST  0x40
#define CC11xx_READ_SINGLE  0x80
#define CC11xx_READ_BURST   0xC0

// Various constants
#define CC11xx_FIFO_SIZE         64     // Rx or Tx FIFO size
#define CC11xx_PACKET_COUNT_SIZE 255    // Packet bytes maximum count

#define MAC_FEC_PARITY_RS            NPAR
#define MAC_HEADER_SIZE             4

#ifndef MAC_UNCODED_PACKET_SIZE
#define MAC_UNCODED_PACKET_SIZE     (CC11xx_PACKET_COUNT_SIZE - MAC_FEC_PARITY_RS)
#endif

#ifndef MAC_PAYLOAD_SIZE
#define MAC_PAYLOAD_SIZE             (MAC_UNCODED_PACKET_SIZE - MAC_HEADER_SIZE)
#endif

#ifndef MAC_ADDITIONAL_INFO
#define MAC_ADDITIONAL_INFO         2
#endif

#define MAC_CSMA_ENABLE

#define CC_SYNC1_DL                    0xD3
#define CC_SYNC0_DL                    0x91

#define CC_SYNC1_UL                    0xD2
#define CC_SYNC0_UL                    0x59

typedef struct cc1101_external_info_s {
    radio_mode_t    mode;
    uint32_t        packet_rx_count;
    uint32_t        packet_errors_corrected;
    uint32_t        packet_tx_count;
    uint32_t         packet_not_tx_count;
    uint8_t         actual_rssi;
    uint8_t         last_rssi;
    uint8_t         last_lqi;
}cc1101_external_info_t;

/* spi structure */
typedef struct spi_parms_s
{
    int      fd;
    int      ret;    /* Ret value of funcking SPI */
    uint8_t  status;
    uint8_t  tx[CC11xx_FIFO_SIZE + 1]; // max 1 command byte + 64 bytes FIFO
    uint8_t  rx[CC11xx_FIFO_SIZE + 1]; // max 1 status byte + 64 bytes FIFO
    uint8_t  len;
} spi_parms_t;

/* Radio parameters */
typedef struct radio_parms_s
{
    uint32_t           f_xtal;        // Crystal frequency (Hz)
    float              freq_tx;       // RF frequency for RX;
    float              freq_rx;       // RF frequency for TX;
    float              f_if;          // IF frequency (Hz)
    float                f_off;
    radio_modulation_t modulation;    // Type of modulation
    rate_t             drate;         // Data rate of the system
    float              mod_index;     // Modulation index Carlson rule
    uint8_t            fec;           // FEC is in use
    uint8_t            whitening;     // Whitening useds
    preamble_t         preamble;      // Preamble count
    sync_word_t        sync_ctl;      // Sync word control
    uint32_t            timeout;          // Timeout for packet CCA
    uint32_t           freq_word;     // Frequency 24 bit word FREQ[23..0]
    uint8_t            chanspc_m;     // Channel spacing mantissa
    uint8_t            chanspc_e;     // Channel spacing exponent
    uint8_t            if_word;       // Intermediate frequency 5 bit word FREQ_IF[4:0]
    uint8_t            drate_m;       // Data rate mantissa
    uint8_t            drate_e;       // Data rate exponent
    uint8_t            chanbw_m;      // Channel bandwidth mantissa
    uint8_t            chanbw_e;      // Channel bandwidth exponent
    uint8_t            deviat_m;      // Deviation mantissa
    uint8_t            deviat_e;      // Deviation exponent
} radio_parms_t;

/* Handler for radio interrupt data */
typedef volatile struct radio_int_data_s
{
    spi_parms_t     *spi_parms;
    radio_parms_t   *radio_parms;
    radio_mode_t    mode;                   // Radio mode (essentially Rx or Tx)
    uint32_t        packet_rx_count;        // Number of packets received since put into action
    uint32_t         packet_rx_corrected;
    uint32_t        packet_tx_count;        // Number of packets sent since put into action
    uint32_t         packet_not_tx_count;    // Number of packets not sent cause of CSMA
    uint8_t         tx_buf[CC11xx_PACKET_COUNT_SIZE]; // Tx buffer
    uint8_t         tx_count;               // Number of bytes in Tx buffer
    uint8_t         rx_buf[CC11xx_PACKET_COUNT_SIZE]; // Rx buffer
    uint8_t         rx_count;               // Number of bytes in Rx buffer
    uint8_t         bytes_remaining;        // Bytes remaining to be read from or written to buffer (composite mode)
    uint8_t         byte_index;             // Current byte index in buffer
    uint8_t         packet_receive;         // Indicates reception of a packet is in progress
    uint8_t         packet_send;            // Indicates transmission of a packet is in progress
} radio_int_data_t;

#ifndef RADIO_PACKET_STRUCT
#define RADIO_PACKET_STRUCT
typedef union __attribute__ ((__packed__)) radio_packet_s {
        uint8_t     raw[MAC_UNCODED_PACKET_SIZE + MAC_ADDITIONAL_INFO];
        /* + 2 which are rssi+lqi */
        struct __attribute__ ((packed)){
            /* protocol information is 2 bytes:
             * * 4 bits protocol ID
             * * 4 bits ESI
             * * 4 bits K value
             * * 4 bits R value
             */
            uint8_t     info_n_esi;
            uint8_t     k_n_r;
            uint8_t     chunk_sequence;
            uint8_t     source_address;
            uint8_t     data[MAC_PAYLOAD_SIZE]; /* --> this shit contains the length... */ /* Inside the chunk */
            /* those two are not appended in transmit mode ofc */
            uint8_t     rssi;
            uint8_t     lqi;
        }fields;
}radio_packet_t;
#endif


void     get_cc1101_statistics(cc1101_external_info_t *cc1101_info);

float   rssi_lna_dbm(uint8_t rssi_dec);
float   rssi_raw_dbm(uint8_t rssi_dec);
float     lqi_status(uint8_t lqi);

int     set_freq_parameters(float freq_rx, float freq_tx, float freq_if, float freq_off, radio_parms_t * radio_parms);
int     set_sync_parameters(preamble_t preamble, sync_word_t sync_word, uint32_t timeout_ms, radio_parms_t * radio_parms);
int     set_packet_parameters(bool fec, bool white, radio_parms_t * radio_parms);
int     set_modulation_parameters(radio_modulation_t mod, rate_t data_rate, float mod_index, radio_parms_t * radio_parms);

void     radio_calibrate(spi_parms_t *spi_parms);

int     init_radio_config(spi_parms_t * spi_parms, radio_parms_t * radio_parms);
int     reconfigure_radio_config(spi_parms_t * spi_parms, radio_parms_t * radio_parms);
/* Used to send a packet with CCA */

void    enable_isr_routine(radio_parms_t * radio_parms);

int     radio_send_packet(radio_packet_t * packet);

void     initialize_cc1101(void);

void     cc1101_work(void);
void     csma_tx_work(void);

#endif
