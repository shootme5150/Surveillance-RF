#ifndef MSPSETUP_H_
#define MSPSETUP_H_
#endif

#include <msp430.h>
#include "int_types.h"

// 8V LNA controls
//Port 5 8V Enables
#define EN_8V_PORT1	P5OUT
#define EN_8V_2		BIT0
#define TX_8V_EN1	BIT1
#define RX_8V_EN1	BIT2
#define TX_8V_EN2	BIT3
#define RX_8V_EN2	BIT4

//Port 3 8V Enables
#define EN_8V_PORT2	P3OUT
#define EN_8V_1		BIT4
#define TX_8V_EN3	BIT2
#define RX_8V_EN3	BIT0
#define TX_8V_EN4	BIT1
#define RX_8V_EN4	BIT3

// Intput address lines
#define ADDR_IN		P6IN
#define ADDR0		(ADDR_IN & BIT0)
#define ADDR1		(ADDR_IN & BIT1)
#define ADDR2		(ADDR_IN & BIT2)
#define ADDR3		(ADDR_IN & BIT3)
#define ADDR4		(ADDR_IN & BIT4)
#define ADDR5		(ADDR_IN & BIT5)
#define ADDR6		(ADDR_IN & BIT6)

// LED outputs
#define ADDR_LEDs	P1OUT
#define ADDR0_LED	BIT0  //(ADDR_OUT & BIT0)
#define ADDR1_LED	BIT1  //(ADDR_OUT & BIT1)
#define ADDR2_LED	BIT2  //(ADDR_OUT & BIT2)
#define ADDR3_LED	BIT3  //(ADDR_OUT & BIT3)

// UART PINS
#define TXMODE              0
#define RXMODE              1

#define MAX_LENGTH          8

#define EN_TX_RX_PORT	P4OUT

// RX Reciever is active low
#define TX_EN_PIN		BIT0

// TX Driver is active high
#define RX_EN_PIN		BIT1

// UART PINS
#define UCRX_PIN		BIT2
#define UCTX_PIN		BIT3

#define TXMODE485           0
#define RXMODE485           1

#define DATA_RS485          0
#define HDR_RS485           1
#define CRC_RS485           2

#define DATA_SPI			0
#define HDR_SPI				1
#define CRC_SPI				2

#define HDR_SIZE            2
#define DATA_SIZE           1
#define KXKY_SIZE			8
#define CRC_SIZE            1

#define WAIT_TXBUF		(UCA1STATW & UCBUSY)

#define SMCLK_BITS		  0x20


// board position address declaration, used to determine if TX or RX is the lowest cell
#define ODD					1
#define EVEN				0

// clock variable, frequency lock loop
#define MCLK_FREQ_MHZ 		20			// MCLK = 20MHz
void Software_Trim();					// Software Trim to get the best DCOFTRIM value

// void power_test();  // 8V powerup test
void startup();
void power_enable(uint8_t tr_mode);  // 8V powerup sequence
void switch_power(uint8_t tr_mode);  // 8V power switch between TX and RX
void adar_config();  // top level adar configuration
void configure_uart();
void configure_spi();
int get_slave_pos();
int get_row(int slave_address);
int get_col(int slave_address);
char get_alignment(int slave_address);
