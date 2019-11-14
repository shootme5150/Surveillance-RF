/* Author: Justin Rhodes
 * Company: Agile RF Systems LLC
 * Description: MFP Surveillance RF uController initialization file
 */

#include <msp430.h>
#include "stdint.h"
#include "stdio.h"
#include "MSP_setup.h"
#include "adar1000.h"

const int pwrup_wait_cycles = 10000;  // delay between power enables, in clock cycles

void startup()
{
		// Configure two FRAM waitstate as required by the device datasheet for MCLK
		// operation at 24MHz(beyond 8MHz) _before_ configuring the clock system.
		FRCTL0 = FRCTLPW | NWAITS_2;

		__bis_SR_register(SCG0);  // disable FLL
		CSCTL3 = SELREF_1 | FLLREFDIV__1;  // FLL reference set to REFOCLK
		CSCTL0 = 0;  // clear DCO and MOD registers
		CSCTL1 = DCORSEL_6 | DCOFTRIMEN;  // Enable DCO Freq Trim and set DCO to 20MHz (DCORSEL_6)
		CSCTL2 = FLLD_0 + 609;  // DCODIV = 20MHz
		CSCTL6 = XTS | XT1HFFREQ_3 | XT1DRIVE_3;  // enable XT1 high-freq mode, >16 MHz to 24 MHz, maximum XT1 drive level

		//__delay_cycles(3);
		__bic_SR_register(SCG0);						// enable FLL
		//Software_Trim();								// Software Trim to get the best DCOFTRIM value

		CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;        // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
														  // default DCOCLKDIV as MCLK and SMCLK source

		// while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked

		PM5CTL0 &= ~LOCKLPM5;  // Disable the GPIO power-on default high-impedance mode
								// to activate previously configured port settings


		adar_config();  // top level adar configuration
		configure_spi();
		configure_uart();
}

// Find the best DCOTRIM setting for FFL stability
void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

/*		int cnt=0;
		do
		{
			cnt++;
		}while (cnt < 100);
*/
        //__delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
                                                           // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}

// configure ADAR control lines
void adar_config()
{
	//Set ADAR1000 Controls to outputs and initialize
	P2DIR |= ADAR1_nEN;
	CS_PORT |= ADAR1_nEN;  // set output high
	P2DIR |= ADAR2_nEN;
	CS_PORT |= ADAR2_nEN;  // set output high

	// Set ADAR physical addresses
	P2DIR |= ADAR_ADDR0 | ADAR_ADDR1;
	CS_PORT &= ~ADAR_ADDR0; // Make ADAR1 Address 00
	CS_PORT |= ADAR_ADDR1;  // Make ADAR2 Address 01

	P3DIR |= ADAR_TnR;
	TR_PORT |= ADAR_TnR;  // Initialize in Tx state
	P3DIR |= RX_LOAD | TX_LOAD;
	TR_PORT &= ~RX_LOAD;  // Set both load lines low
	TR_PORT &= ~TX_LOAD;

	// Initialize inputs from Master
	P2DIR &= ~TnR_IN;

	// Address intput lines
	P6DIR &= ~BIT0;
	P6DIR &= ~BIT1;
	P6DIR &= ~BIT2;
	P6DIR &= ~BIT3;
	P6DIR &= ~BIT4;
	P6DIR &= ~BIT5;
	P6DIR &= ~BIT6;

	//Address LED outputs
	P1DIR |= BIT0;
	P1DIR |= BIT1;
	P1DIR |= BIT2;
	P1DIR |= BIT3;
}

void configure_spi()
{
	// P4.5 is B1 CLK, P4.6 is B1 MOSI, P4.7 is MISO
	P4SEL0 |= BIT5 | BIT6 | BIT7;			 // set 3-SPI pin as second function

	// Latches are set in adar config function

	UCB1CTLW0 |= UCSWRST;					 // **Put state machine in reset**
	UCB1CTLW0 |= UCMST;						// Master mode
	UCB1CTLW0 |= UCSYNC|UCCKPL|UCMSB;   // 3-pin, 8-bit SPI master
											  // Clock polarity high, MSB
	UCB1CTLW0 |= UCSSEL__SMCLK;				// Select ACLK
	UCB1BR0 = 0x20;						   // BRCLK = SMCLK/4
	UCB1BR1 = 0;							  //
//	UCB1MCTLW = 0;							// No modulation

//	UCB1IE = UCRXIE;
	UCB1CTLW0 &= ~UCSWRST;					// **Initialize USCI state machine**
	UCB1IE &= ~UCRXIE;						 // Disable USCI_A1 RX interrupt
	UCB1IE &= ~UCTXIE;

	// UCA0 is a slave, SPI connected to main controller
	P1SEL0 |= BIT5 | BIT6 | BIT7;			 // set 3-SPI pin as second function

	UCA0CTLW0 |= UCSWRST;					 // **Put state machine in reset**
	UCA0CTLW0 &= ~UCMST;				// Slave mode
	UCA0CTLW0 |= UCSYNC|UCCKPL|UCMSB;   // 3-pin, 8-bit SPI
										// Clock polarity high, MSB first
	UCA0CTLW0 |= UCSSEL__SMCLK;			// Select SMCLK
	UCA0BR0 = 0x08;						// BRCLK = SMCLK/4
	UCA0BR1 = 0;						//

	UCA0MCTLW = 0;                            // No modulation
	UCA0CTLW0 &= ~UCSWRST;				// **Initialize USCI state machine**
	UCA0IE &= ~UCRXIE;					// Enable USCI_A0 RX interrupt
	UCA0IE &= ~UCTXIE;					// Disable TX interrupt

	return;
}


void configure_uart()
{
	/* P4.0 and P4.1 are enable pins
	 set the direction to output
	 */
	P4DIR |= RX_EN_PIN | TX_EN_PIN;
	// RX Reciever is active low so start the bit high
	EN_TX_RX_PORT |= RX_EN_PIN;
	// TX Driver is active high so start bit low
	EN_TX_RX_PORT &= ~TX_EN_PIN;

	// P4.3 (TX) and P4.2 (RX) are UART Pins
	P4SEL0 |= UCTX_PIN | UCRX_PIN;

	// put in software reset
	UCA1CTLW0 |= UCSWRST;
	// UCA1 default settings, see Table 22-8 in users guide
	// Parity disabled (odd)
	// LSB first
	// 8-bit data transfer
	// one stop bit
	// Asynchronous UART mode
	// select SMCLK
	UCA1CTLW0 |= UCSSEL__SMCLK;
	// TODO Figure out addres bits, break conditions, parity, etc

	// set baud rate -> 2.4 MHz / 9600 = 250 (0xFA)
	// The valid baud-rate control range is 3 < UxBR < 0FFFFh, where UxBR = {UxBR1+UxBR0}.
	UCA1BR0 = 4;                             // 20M/5M
	UCA1BR1 = 0x00;
	UCA1MCTLW = 0x00;
	// enable USART Module
	UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
	UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}


// power up the 8V lines in a controlled manner initially
void power_enable(uint8_t tr_mode)
{
	// Configure 8V LNA controls
	// Set 8V Port 1 enables to outputs
	P5DIR |= EN_8V_2;
	P5DIR |= TX_8V_EN1 | TX_8V_EN2;
	P5DIR |= RX_8V_EN1 | RX_8V_EN2;

	// Set 8V Port 1 enables low
	EN_8V_PORT1 &= ~EN_8V_2;
	EN_8V_PORT1 &= ~TX_8V_EN1;
	EN_8V_PORT1 &= ~RX_8V_EN1;
	EN_8V_PORT1 &= ~TX_8V_EN2;
	EN_8V_PORT1 &= ~RX_8V_EN2;

	// Set 8V Port 2 enables to outputs
	P3DIR |= EN_8V_1;
	P3DIR |= TX_8V_EN3 | TX_8V_EN4;
	P3DIR |= RX_8V_EN3 | RX_8V_EN4;

	// Set 8V Port 2 enables low
	EN_8V_PORT2 &= ~EN_8V_1;
	EN_8V_PORT2 &= ~TX_8V_EN3;
	EN_8V_PORT2 &= ~TX_8V_EN4;
	EN_8V_PORT2 &= ~RX_8V_EN3;
	EN_8V_PORT2 &= ~RX_8V_EN4;

	// enable 8V LDOs
	//__delay_cycles(pwrup_wait_cycles);
	EN_8V_PORT1 |= EN_8V_2;
	//__delay_cycles(pwrup_wait_cycles);
	EN_8V_PORT2 |= EN_8V_1;
	//__delay_cycles(pwrup_wait_cycles);

	// initiate switches based on initial state
	switch(tr_mode)
	{
		case TX:
			// open switches for RX
			EN_8V_PORT1 &= ~RX_8V_EN1;
			EN_8V_PORT1 &= ~RX_8V_EN2;
			EN_8V_PORT2 &= ~RX_8V_EN3;
			EN_8V_PORT2 &= ~RX_8V_EN4;

            // close switch for TX1,3
			EN_8V_PORT1 |= TX_8V_EN1;
			EN_8V_PORT2 |= TX_8V_EN3;
			//__delay_cycles(pwrup_wait_cycles);

			// close switch for TX2,4
			EN_8V_PORT1 |= TX_8V_EN2;
			EN_8V_PORT2 |= TX_8V_EN4;
			//__delay_cycles(pwrup_wait_cycles);
		break;

		case RX:
			// open switches for RX
			EN_8V_PORT1 &= ~TX_8V_EN1;
			EN_8V_PORT1 &= ~TX_8V_EN2;
			EN_8V_PORT2 &= ~TX_8V_EN3;
			EN_8V_PORT2 &= ~TX_8V_EN4;

			// close switch for RX1,3
			EN_8V_PORT1 |= RX_8V_EN1;
			EN_8V_PORT2 |= RX_8V_EN3;
			//__delay_cycles(pwrup_wait_cycles);

			// close switch for RX2,4
			EN_8V_PORT1 |= RX_8V_EN2;
			EN_8V_PORT2 |= RX_8V_EN4;
		break;
	}

}

// enable and disable the proper 8V switches to change between TX and RX
// NOTE!!!  This may require short delays between switching to ensure nothing crashes
void switch_power(uint8_t tr_mode)
{
	switch(tr_mode)
	{
		case TX:
			// open switches for RX and close switches for TX.  Stagger these between the two sources to reduce instantaneous current draw
			EN_8V_PORT1 &= ~RX_8V_EN1;
			EN_8V_PORT1 |= TX_8V_EN1;

			EN_8V_PORT2 &= ~RX_8V_EN3;
			EN_8V_PORT2 |= TX_8V_EN3;

			EN_8V_PORT1 &= ~RX_8V_EN2;
			EN_8V_PORT1 |= TX_8V_EN2;

			EN_8V_PORT2 &= ~RX_8V_EN4;
			EN_8V_PORT2 |= TX_8V_EN4;

		break;

		case RX:
			// open switches for RX and close switches for TX.  Stagger these between the two sources to reduce instantaneous current draw
			EN_8V_PORT1 &= ~TX_8V_EN1;
			EN_8V_PORT1 |= RX_8V_EN1;

			EN_8V_PORT1 &= ~TX_8V_EN2;
			EN_8V_PORT1 |= RX_8V_EN2;

			EN_8V_PORT2 &= ~TX_8V_EN3;
			EN_8V_PORT2 |= RX_8V_EN3;

			EN_8V_PORT2 &= ~TX_8V_EN4;
			EN_8V_PORT2 |= RX_8V_EN4;

		break;
	}
}

// Determine absolute board position within the total array
int get_slave_pos()
{
	// Read in address and set lower LEDs for verification
	int slave_addr = ADDR0 | ADDR1 | ADDR2 | ADDR3 | ADDR4 | ADDR5 | ADDR6;
	ADDR_LEDs = slave_addr & 0x0F;
	return slave_addr;
}

// Determine x position of board within array (column)
int get_col(int slave_address)
{
	int master_pos = (slave_address & 0x70) >> 4;
	int slave_pos = slave_address &0x0F;
	int col_pos;

	// determine which half of the array we're in
	if ((master_pos == 0) | (master_pos == 2))
	{
		col_pos = slave_pos;
	} else {
		col_pos = slave_pos + 16;
	}

	return (col_pos+1);
}
char get_alignment(int slave_address)
{
	char alignment;
	// determine whether column number is even or odd (due to checkerboard)
	if (ADDR0)
		alignment = ODD;
	else
		alignment = EVEN;

	return alignment;
}


// Determine Y position of board (first cell) within array (row)
int get_row(int slave_address)
{
	int module_pos = (slave_address & 0x70) >> 4;
	int row_pos;

	switch(module_pos)
	{
		case 0:
			row_pos = 1;
		break;

		case 1:
			row_pos = 1;
		break;

		case 2:
			row_pos = 17;
		break;

		case 3:
			row_pos = 17;
		break;
	}
	return row_pos;
}

/*
// sequentially bring up power and toggle all pertinent banks to look for overloads.  Use with debug and breakpoints.
void power_test()
{
	// Configure 8V LNA controls
	// Set 8V Port 1 enables to outputs
	P5DIR |= EN_8V_2;
	P5DIR |= TX_8V_EN1 | TX_8V_EN2;
	P5DIR |= RX_8V_EN1 | RX_8V_EN2;

	// Set 8V Port 1 enables low (disable)
	EN_8V_PORT1 &= ~EN_8V_2;
	EN_8V_PORT1 &= ~TX_8V_EN1;
	EN_8V_PORT1 &= ~RX_8V_EN1;
	EN_8V_PORT1 &= ~TX_8V_EN2;
	EN_8V_PORT1 &= ~RX_8V_EN2;

	// Set 8V Port 2 enables to outputs
	P3DIR |= EN_8V_1;
	P3DIR |= TX_8V_EN3 | TX_8V_EN4;
	P3DIR |= RX_8V_EN3 | RX_8V_EN4;

	// Set 8V Port 2 enables low (disable)
	EN_8V_PORT2 &= ~EN_8V_1;
	EN_8V_PORT2 &= ~TX_8V_EN3;
	EN_8V_PORT2 &= ~TX_8V_EN4;
	EN_8V_PORT2 &= ~RX_8V_EN3;
	EN_8V_PORT2 &= ~RX_8V_EN4;

	// cycle 8V LDOs one at a time
	//__delay_cycles(pwrup_wait_cycles);
	EN_8V_PORT1 |= EN_8V_2;  // enable
	EN_8V_PORT1 &= ~EN_8V_2;  // disable

	//__delay_cycles(pwrup_wait_cycles);
	EN_8V_PORT2 |= EN_8V_1;  //enable 8V
	EN_8V_PORT2 &= ~EN_8V_1;  // disable 8V

	// toggle first bank of MAAM 8V switches
    EN_8V_PORT2 |= EN_8V_1;  //enable 8V
	//__delay_cycles(pwrup_wait_cycles);
	// close switch for TX1
	EN_8V_PORT1 |= TX_8V_EN1;
	EN_8V_PORT1 &= ~TX_8V_EN1;

	//__delay_cycles(pwrup_wait_cycles);
	// close switch for RX1
	EN_8V_PORT1 |= RX_8V_EN1;
	EN_8V_PORT1 &= ~RX_8V_EN1;

	//__delay_cycles(pwrup_wait_cycles);
	// close switch for TX2
	EN_8V_PORT1 |= TX_8V_EN2;
	EN_8V_PORT1 &= ~TX_8V_EN2;

	//__delay_cycles(pwrup_wait_cycles);
	// close switch for RX2
	EN_8V_PORT1 |= RX_8V_EN2;
	EN_8V_PORT1 &= ~RX_8V_EN2;

	EN_8V_PORT2 &= ~EN_8V_1;  // disable 8V

	// toggle second bank of MAAM 8V switches
	//__delay_cycles(pwrup_wait_cycles);
	EN_8V_PORT1 |= EN_8V_2;  //enable 8V

	//__delay_cycles(pwrup_wait_cycles);
	// close switch for TX3
	EN_8V_PORT2 |= TX_8V_EN3;
	EN_8V_PORT2 &= ~TX_8V_EN3;

	//__delay_cycles(pwrup_wait_cycles);
	// close switch for RX3
	EN_8V_PORT2 |= RX_8V_EN3;
	EN_8V_PORT2 &= ~RX_8V_EN3;

	//__delay_cycles(pwrup_wait_cycles);
	// close switch for TX4
	EN_8V_PORT2 |= TX_8V_EN4;
	EN_8V_PORT2 &= ~TX_8V_EN4;

	//__delay_cycles(pwrup_wait_cycles);
	// close switch for RX4
	EN_8V_PORT2 |= RX_8V_EN4;
	EN_8V_PORT2 &= ~RX_8V_EN4;

	EN_8V_PORT1 &= ~EN_8V_2;  // disable 8V

}
*/
