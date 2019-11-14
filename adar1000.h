/*
 * adar1000.h
 *
 *  Created on: Jul 23, 2019
 *      Author: User
 */

#ifndef ADAR1000_H_
#define ADAR1000_H_

#include <msp430.h>
#include "int_types.h"


/* ------------ device and application specific defines, for SPI Ports and Pins -------------------- */

#define SPI_RX_BUFF         UCB1RXBUF
#define SPI_TX_BUFF         UCB1TXBUF
#define WAIT_SPI            UCB1STATW & UCBUSY

//ADAR1000 Controls
#define CS_PORT		P2OUT
#define ADAR1_nEN	BIT3
#define ADAR2_nEN	BIT2
#define ADAR_ADDR0	BIT1
#define ADAR_ADDR1	BIT0
#define TR_PORT		P3OUT
#define ADAR_TnR	BIT7
#define RX_LOAD		BIT6
#define TX_LOAD		BIT5

// Communication definitions from master (SPI)
#define WAIT_RX_FLAG		UCA0IFG & UCRXIFG
#define WAIT_RX_SPI			UCA0STATW & UCBUSY

// Definitions for reset and initialization commands
#define CONFIG_BIT		BIT7
#define RST_BIT 		BIT6
#define INIT_BIT		BIT5
#define ALL_RST_INIT	BIT4

// Random pattern to reset communications - 10010101
#define RESET_COMMS		0x0095

// Definitions for Amplitude or Phase set commands
// First Byte
#define CONST_BIT		BIT6
#define AMP_nPHASE_BIT	BIT5
#define ADDR_BITS		(BIT4 | BIT3 | BIT2 | BIT1 | BIT0)
/*  slave definitions from master
#define SLAVE1		0x00
#define SLAVE2		0x01
#define SLAVE3		0x02
#define SLAVE4		0x03
#define SLAVE5		0x04
#define SLAVE6		0x05
#define SLAVE7		0x06
#define SLAVE8		0x07
#define SLAVE9		0x08
#define SLAVE10		0x09
#define SLAVE11		0x0A
#define SLAVE12		0x0B
#define SLAVE13		0x0C
#define SLAVE14		0x0D
#define SLAVE15		0x0E
#define SLAVE16		0x0F
#define ALL_SLAVES	0x10
*/

// second byte
#define TR_BITS			(BIT7 | BIT6)		// Data to be applied to Tx, Rx, or all?
#define RX				0x00
#define TX				0x01
#define TRX				0x02

#define CH_BITS			(BIT5 | BIT4 | BIT3 | BIT2)	// Data to be applied to specific channel, or set of channels
// ADAR channel addressing
#define CH1				0x00
#define CH2				0x01
#define CH3				0x02
#define CH4				0x03
#define CH5				0x04
#define CH6				0x05
#define CH7				0x06
#define CH8				0x07
#define ALL_CH			0x08
#define CH1_4			0x09
#define CH5_8			0x0A

#define AMP_PHASE_BIT	BIT2			// Declares amplitude or phase
#define PHASE_MODE		0
#define AMP_MODE		1

// Definitions for Amplitude or Phase set commands
// definitions for gain setting bits
#define GAIN_BITS		(BIT1 | BIT0)	// Update gain setting, if needed
#define NO_CHANGE		0x00
#define LOW_GAIN		0x01
#define NOM_GAIN		0x02
#define HI_GAIN			0x03

// Definitions for reset and initialization commands
#define CONFIG_BIT		BIT7
#define RST_BIT 		BIT6
#define INIT_BIT		BIT5
#define ALL_RST_INIT	BIT4


#define direct			0
#define KxKy			1

// Inputs from Master
#define TnR_IN		BIT5
#define CS_IN		BIT5

#define TR_SPI_CTL          0
#define TR_TTL_CTL          1

/* ------------ Function Definitions -------------------- */

// function to write a register on the ADAR1000
uint8_t adar_reg_write(uint16_t address, uint8_t data, uint16_t device);

// function to read a register on the ADAR1000
uint8_t adar_reg_read(uint16_t address, uint16_t device);

// function to set the phase for RX/TX on channel 1-4, at the specified phase state
uint8_t set_phase_by_channel(uint8_t tr, uint8_t channel, uint8_t phase, uint16_t device);

// initializes the ADAR1000 with basic universal purpose settings
void init_ADAR1000(uint16_t device);

// resets the ADAR1000 to reset low power state
void reset_ADAR1000(uint16_t device);

// configure the ADAR1000 to work in SPI control rather than off of values in memory
void enable_spi_control(uint16_t device);

// control TR settings and mode via spi
void spi_TR_mode(uint8_t mode, uint8_t spi_ctl, uint16_t device);

// control TR mode via GPIO pin
void ttl_TR_mode(uint8_t mode);

void spi_load_working_reg(uint8_t mode, uint16_t device);

uint8_t initialize_channel(uint8_t channel, uint8_t tr, uint16_t device);

void select_device(uint16_t device);

/* ----------------- useful function definitions -------------------- */

/* ------------ ADAR1000 Memory Addressing -------------------- */

// MSB of address is R/W Bit
#define READ_ADDR           0x8000
#define WRITE_ADDR          0x0000

// Device Address Bits
#define AD1                 0x4000
#define AD0                 0x2000
#define ADAR0				0x0000  // 00
#define ADAR1				0x2000  // 01
#define ADAR2				0x4000  // 10
#define ADAR3				0x6000  // 11

// All Devices Address
#define ALL_ADAR			0x800

// SPI ADDRESS BIT 12 for CTL bits or BEAM State Memory
#define CTL_REG_FLAG        0x0000
#define BEAM_MEM_FLAG       0x1000

// Transmit/Recieve Flag Define
#define RX_MODE_MASK        0x0000
#define TX_MODE_MASK        0x0800

// channel masks for position settings
#define CH1_ADDR_MASK       0x0000
#define CH2_ADDR_MASK       0x0004
#define CH3_ADDR_MASK       0x0008
#define CH4_ADDR_MASK       0x000C

// register byte masks for phase
#define LOW_BYTE          0x0000
#define MID_BYTE          0x0001
#define HIGH_BYTE         0x0002

// register byte masks for RX bias settings
#define RX_BYTE_1            0x0000
#define RX_BYTE_2            0x0001
#define RX_BYTE_3            0x0004
#define RX_BYTE_4            0x0005

// register byte masks for TX bias settings
#define TX_BYTE_1            0x0000
#define TX_BYTE_2            0x0001
#define TX_BYTE_3            0x0002
#define TX_BYTE_4            0x0004
#define TX_BYTE_5            0x0005
#define TX_BYTE_6            0x0006
#define TX_BYTE_7            0x0008
#define TX_BYTE_8            0x0009
#define TX_BYTE_9            0x000C
#define TX_BYTE_10           0x000D

/* -------------------------- MACROS ------------------------------------ */

// Position bits mask shifts 6 bit number so that MSB in BIT10
#define POSITION_BITS(pos) (pos << 4)

// bias setting is added to last position setting
#define BIAS_SETTING(bias) (POSITION_BITS(120 + bias))

// phase index from phase is scaled by 128 positions (2^7) in 360 deg, so divide by 45 and multiply by 16
#define PHASE_INDEX(phase) (((int) phase/45) << 4)

// from vset in milivolts, the Bias Ouput index is vset_mv/4800*2^8
#define BIAS_INDEX(vset_mv) ((uint16_t) ((vset_mv << 8)/4800))

// VGA Gain bits are in bits 6-3, so shift left by three
#define VGA_GAIN_BITS(gain) (gain << 3)


/* ----------- Control Register Address Definitions ------------------ */
#define INTERFACE_CONFIG_A      0x000
#define INTERFACE_CONFIG_B      0x001
#define DEV_CONFIG              0x002
#define CHIP_TYPE               0x003
#define PRODUCT_ID_H            0x004
#define PRODUCT_ID_L            0x005
#define SCRATCH_PAD             0x00A
#define SPI_REV                 0x00B
#define VENDOR_ID_H             0x00C
#define VENDOR_ID_L             0x00D
#define TRANSFER_REG            0x00F
#define CH1_RX_GAIN             0x010
#define CH2_RX_GAIN             0x011
#define CH3_RX_GAIN             0x012
#define CH4_RX_GAIN             0x013
#define CH1_RX_PHASE_I          0x014
#define CH1_RX_PHASE_Q          0x015
#define CH2_RX_PHASE_I          0x016
#define CH2_RX_PHASE_Q          0x017
#define CH3_RX_PHASE_I          0x018
#define CH3_RX_PHASE_Q          0x019
#define CH4_RX_PHASE_I          0x01A
#define CH4_RX_PHASE_Q          0x01B
#define CH1_TX_GAIN             0x01C
#define CH2_TX_GAIN             0x01D
#define CH3_TX_GAIN             0x01E
#define CH4_TX_GAIN             0x01F
#define CH1_TX_PHASE_I          0x020
#define CH1_TX_PHASE_Q          0x021
#define CH2_TX_PHASE_I          0x022
#define CH2_TX_PHASE_Q          0x023
#define CH3_TX_PHASE_I          0x024
#define CH3_TX_PHASE_Q          0x025
#define CH4_TX_PHASE_I          0x026
#define CH4_TX_PHASE_Q          0x027
#define LD_WRK_REGS             0x028
#define CH1_PA_BIAS_ON          0x029
#define CH2_PA_BIAS_ON          0x02A
#define CH3_PA_BIAS_ON          0x02B
#define CH4_PA_BIAS_ON          0x02C
#define LNA_BIAS_ON             0x02D
#define RX_ENABLES              0x02E
#define TX_ENABLES              0x02F
#define MISC_ENABLES            0x030
#define SW_CTRL                 0x031
#define ADC_CTRL                0x032
#define ADC_OUTPUT              0x033
#define BIAS_CURRENT_RX_LNA     0x034
#define BIAS_CURRENT_RX         0x035
#define BIAS_CURRENT_TX         0x036
#define BIAS_CURRENT_TX_DRV     0x037
#define MEM_CTRL                0x038
#define RX_CHX_MEM              0x039
#define TX_CHX_MEM              0x03A
#define RX_CH1_MEM              0x03D
#define RX_CH2_MEM              0x03E
#define RX_CH3_MEM              0x03F
#define RX_CH4_MEM              0x040
#define TX_CH1_MEM              0x041
#define TX_CH2_MEM              0x042
#define TX_CH3_MEM              0x043
#define TX_CH4_MEM              0x044
#define REV_ID                  0x045
#define CH1_PA_BIAS_OFF         0x046
#define CH2_PA_BIAS_OFF         0x047
#define CH3_PA_BIAS_OFF         0x048
#define CH4_PA_BIAS_OFF         0x049
#define LNA_BIAS_OFF            0x04A
#define TX_TO_RX_DELAY_CTRL     0x04B
#define RX_TO_TX_DELAY_CTRL     0x04C
#define TX_BEAM_STEP_START      0x04D
#define TX_BEAM_STEP_STOP       0x04E
#define RX_BEAM_STEP_START      0x04F
#define RX_BEAM_STEP_STOP       0x050
#define RX_BIAS_RAM_CTL         0x051
#define TX_BIAS_RAM_CTL         0x052
#define LDO_TRIM_CTL_0          0x400
#define LDO_TRIM_CTL_1          0x401

/* ----------- Control Register BIT Definitions ------------------*/
// memory control registers defines
#define RAM_PHASE_FETCH         BIT7
#define RAM_BIAS_FETCH          BIT3

// phase polarity defines
#define PHASE_POLARITY_POS      BIT5
#define PHASE_POLARITY_NEG      0x00

// config A register defines (0x000)
#define SOFTRESET               BIT7 | BIT0
#define LSB_FIRST               BIT6 | BIT1
#define ADDR_ASCN               BIT5 | BIT2
#define SDOACTIVE               BIT4 | BIT3

// LDO trim control register (0x401)
#define LDO_TRIM_SEL            0x02

// MISC_ENABLES control register defines (0x030)
#define SW_DRV_TR_MODE_SEL      BIT7       // Transmit/Receive Output Driver Select. If 0, TR_SW_NEG is enabled, and if1, TR_SW_POS is enabled.
#define BIAS_CTRL               BIT6       // External Amplifier Bias Control. If 0, DACs assume the on register values. If 1, DACs vary with device mode (transmit and receive).
#define BIAS_EN                 BIT5       // Enables PA and LNA Bias DACs. 0 = enabled.
#define LNA_BIAS_OUT_EN         BIT4       // Enables Output of LNA Bias DAC. 0 = open and 1 = bias connected.
#define CH1_DET_EN              BIT3       // Enables Channel 1 Power Detector
#define CH2_DET_EN              BIT2       // Enables Channel 2 Power Detector
#define CH3_DET_EN              BIT1       // Enables Channel 3 Power Detector
#define CH4_DET_EN              BIT0       // Enables Channel 4 Power Detector
#define ALL_CH_DET_EN           0x0F       // Enables detector for channels 1-4

// MEM_CTL Register Defines (0x38)
#define SCAN_MODE_EN            BIT7       // Scan Mode Enable
#define BEAM_RAM_BYPASS         BIT6       // Bypass RAM and Load Beam Position Settings from SPI
#define BIAS_RAM_BYPASS         BIT5       // Bypass RAM and Load Bias Position Settings from SPI
#define TX_BEAM_STEP_EN         BIT3       // Sequentially Step Through Stored Transmit Beam Positions
#define RX_BEAM_STEP_EN         BIT2       // Sequentially Step Through Stored Receive Beam Positions
#define TX_CHX_RAM_BYPASS       BIT1       // Bypass RAM for Transmit Channels
#define RX_CHX_RAM_BYPASS       BIT0       // Bypass RAM for Receive Channels

// SW_CTL Register Defines (0x31)
#define SW_DRV_TR_STATE         BIT7       // Controls Sense of Transmit/Receive Switch Driver Output. If 0, the driver outputs 0 V in receive mode.
#define TX_EN                   BIT6       // Enables Transmit Channel Subcircuits when Under SPI Control. 1 = enabled.
#define RX_EN                   BIT5       // Enables Receive Channel Subcircuits when Under SPI Control. 1 = enabled.
#define SW_DRV_EN_TR            BIT4       // Enables Switch Driver for External Transmit/Receive Switch. 1 = enabled.
#define SW_DRV_EN_POL           BIT3       // Enables Switch Driver for External Polarization Switch. 1 = enabled.
#define TR_SOURCE               BIT2       // Source for Transmit/Receive Control. 0 = TR_SPI, 1 = TR input
#define TR_SPI                  BIT1       // State of SPI Control. 0 = receive and 1 = transmit
#define POL                     BIT0       // Control for External Polarity Switch Drivers. 0 = outputs 0 V, and 1 = outputs -5 V, if switch is enabled.

// TX_ENABLES Register Defines (0x2F)
#define CH1_TX_EN               BIT6       // Enables Transmit Channel 1 Subcircuits
#define CH2_TX_EN               BIT5       // Enables Transmit Channel 2 Subcircuits
#define CH3_TX_EN               BIT4       // Enables Transmit Channel 3 Subcircuits
#define CH4_TX_EN               BIT3       // Enables Transmit Channel 4 Subcircuits
#define TX_DRV_EN               BIT2       // Enables the Transmit Channel Drivers
#define TX_VM_EN                BIT1       // Enables the Transmit Channel Vector Modulators
#define TX_VGA_EN               BIT0       // Enables the Transmit Channel VGAs
#define ALL_CH_TX_EN            0x78       // Enables all Transmit Channels (1-4)

// attenuation bit setting for RX/TX, all channels
#define ATTN_SETTING            BIT7	// Attenuator Setting (1 disables attenuator)
// Max Gain Macro Define for RX/TX, All Channels
#define MAX_GAIN                0x7F

// RX_ENABLES Register Defines (0x2E)
#define CH1_RX_EN               BIT6       // Enables Receive Channel 1 Subcircuits
#define CH2_RX_EN               BIT5       // Enables Receive Channel 2 Subcircuits
#define CH3_RX_EN               BIT4       // Enables Receive Channel 3 Subcircuits
#define CH4_RX_EN               BIT3       // Enables Receive Channel 4 Subcircuits
#define RX_LNA_EN               BIT2       // Enables the Receive Channel LNAs
#define RX_VM_EN                BIT1       // Enables the Receive Channel Vector Modulators
#define RX_VGA_EN               BIT0       // Enables the Receive Channel VGAs
#define ALL_CH_RX_EN            0x78       // Enables all Receive Channels (1-4)

/* Gain Nominal and Low Power Settings */

// RX LNA Gain Settings (0x34) B[3:0]
#define LNA_BIAS_NOM            0x8
#define LNA_BIAS_LOW            0x5
// RX Vector Modulator Gain Settings (0x35) B[2:0]
#define RX_VM_BIAS_NOM          0x5
#define RX_VM_BIAS_LOW          0x2
// RX VGA Gain settings (0x35) B[6:3]
#define RX_VGA_BIAS_NOM         (0xA << 3)
#define RX_VGA_BIAS_LOW         (0x3 << 3)
// TX Vector Modulator Gain Settings (0x36) B[2:0]
#define TX_VM_BIAS_NOM          0x5
#define TX_VM_BIAS_LOW          0x2
// TX VGA Gain Settings (0x36) B[2:0]
#define TX_VGA_BIAS_NOM         (0x5 << 3)
//TX Driver Gain settings (0x37) B[2:0]
#define TX_DRV_BIAS_NOM         0x6
#define TX_DRV_BIAS_LOW         0x3

// load working registers commands (0x28) B[1:0]
#define LDTX_OVERRIDE           0x2
#define LDRX_OVERRIDE           0x1



/* ------------------ Look-up tables ------------------------------ */

// look-up tables for ADAR channel alignment, defining which ADAR channel is associate with which row
static const uint8_t EVEN_ADDRESS_CH[] = {3,4,1,2,3,4,1,2};		// RF1 channel sequence
static const uint8_t ODD_ADDRESS_CH[] = {2,1,4,3,2,1,4,3};		// RF2 channel sequence

// Look-up tables for In phase and quadrature gain values 7 bit phase control ~2.2 degree phase increments
static const uint8_t VI_PHASES[] = {0x3F,0x3F,0x3F,0x3F,0x3F,0x3E,0x3E,0x3D,0x3D,0x3C,0x3C,0x3B,0x3A,0x39,0x38,0x37,         // 0   to  42.1875 deg
                                    0x36,0x35,0x34,0x33,0x32,0x30,0x2F,0x2E,0x2C,0x2B,0x2A,0x28,0x27,0x25,0x24,0x22,         // 45  to  87.1875 deg
                                    0x21,0x01,0x03,0x04,0x06,0x07,0x08,0x0A,0x0B,0x0D,0x0E,0x0F,0x11,0x12,0x13,0x14,         // 90  to 132.1875 deg
                                    0x16,0x17,0x18,0x19,0x19,0x1A,0x1B,0x1C,0x1C,0x1D,0x1E,0x1E,0x1E,0x1F,0x1F,0x1F,         // 135 to 177.1875 deg
                                    0x1F,0x1F,0x1F,0x1F,0x1F,0x1E,0x1E,0x1D,0x1D,0x1C,0x1C,0x1B,0x1A,0x19,0x18,0x17,         // 180 to 222.1875 deg
                                    0x16,0x15,0x14,0x13,0x12,0x10,0x0F,0x0E,0x0C,0x0B,0x0A,0x08,0x07,0x05,0x04,0x02,         // 225 to 267.1875 deg
                                    0x01,0x21,0x23,0x24,0x26,0x27,0x28,0x2A,0x2B,0x2D,0x2E,0x2F,0x31,0x32,0x33,0x34,         // 270 to 312.1875 deg
                                    0x36,0x37,0x38,0x39,0x39,0x3A,0x3B,0x3C,0x3C,0x3D,0x3E,0x3E,0x3E,0x3F,0x3F,0x3F};        // 315 to 357.1875 deg

static const uint8_t VQ_PHASES[] = {0x20,0x21,0x23,0x24,0x26,0x27,0x28,0x2A,0x2B,0x2D,0x2E,0x2F,0x30,0x31,0x33,0x34,         // 0   to  42.1875 deg
                                    0x35,0x36,0x37,0x38,0x38,0x39,0x3A,0x3A,0x3B,0x3C,0x3C,0x3C,0x3D,0x3D,0x3D,0x3D,         // 45  to  87.1875 deg
                                    0x3D,0x3D,0x3D,0x3D,0x3D,0x3C,0x3C,0x3C,0x3B,0x3A,0x3A,0x39,0x38,0x38,0x37,0x36,         // 90  to 132.1875 deg
                                    0x35,0x34,0x33,0x31,0x30,0x2F,0x2E,0x2D,0x2B,0x2A,0x28,0x27,0x26,0x24,0x23,0x21,         // 135 to 177.1875 deg
                                    0x20,0x01,0x03,0x04,0x06,0x07,0x08,0x0A,0x0B,0x0D,0x0E,0x0F,0x10,0x11,0x13,0x14,         // 180 to 222.1875 deg
                                    0x15,0x16,0x17,0x18,0x18,0x19,0x1A,0x1A,0x1B,0x1C,0x1C,0x1C,0x1D,0x1D,0x1D,0x1D,         // 225 to 267.1875 deg
                                    0x1D,0x1D,0x1D,0x1D,0x1D,0x1C,0x1C,0x1C,0x1B,0x1A,0x1A,0x19,0x18,0x18,0x17,0x16,         // 270 to 312.1875 deg
                                    0x15,0x14,0x13,0x11,0x10,0x0F,0x0E,0x0D,0x0B,0x0A,0x08,0x07,0x06,0x04,0x03,0x01};        // 315 to 357.1875 deg

// Look up tables for the register address for channel, V/I, T/R
static const uint16_t PHASE_ADDR_I[] = {CH1_RX_PHASE_I, CH2_RX_PHASE_I, CH3_RX_PHASE_I, CH4_RX_PHASE_I,
                                        CH1_TX_PHASE_I, CH2_TX_PHASE_I, CH3_TX_PHASE_I, CH4_TX_PHASE_I };

static const uint16_t PHASE_ADDR_Q[] =  {CH1_RX_PHASE_Q, CH2_RX_PHASE_Q, CH3_RX_PHASE_Q, CH4_RX_PHASE_Q,
                                         CH1_TX_PHASE_Q, CH2_TX_PHASE_Q, CH3_TX_PHASE_Q, CH4_TX_PHASE_Q };

static const uint8_t ENABLES[] = {CH1_RX_EN,CH2_RX_EN,CH3_RX_EN,CH4_RX_EN,ALL_CH_RX_EN};

#endif /* ADAR1000_H_ */
