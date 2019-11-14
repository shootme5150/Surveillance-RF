/* Author: Justin Rhodes
 * Company: Agile RF Systems LLC
 * Description: Top level file for MFP Surveillance Antenna uController
 */

#include <msp430.h>
#include "stdint.h"
#include "stdio.h"
#include "MSP_setup.h"
#include "adar1000.h"

void set_mode_RS485(int mode);
void clock_test();
void set_ADARs(uint8_t tr_mode, uint8_t rx_amp_data, uint8_t tx_amp_data, uint8_t rx_phase_data, uint8_t tx_phase_data);
void PhaseState(float kx, float ky);
void decode_SPI();
void spi_writeKxKy(float kx, float ky);

//void build_int_float(uint32_t data[], uint8_t bytes);

int const_setting = 0;  //set this to enable constant data setting, clear for UART data passing
volatile uint8_t tx_amp_data = ATTN_SETTING | 0X7F;
volatile uint8_t rx_amp_data = ATTN_SETTING | 0X7F;
volatile uint8_t tx_phase_data;
volatile uint8_t rx_phase_data;

volatile uint8_t mode_byte_count = 0;
volatile uint8_t msg_mode = HDR_SPI;

volatile uint8_t tr_mode = RX;
volatile uint8_t ch_select = CH1;
volatile uint8_t ch_range = 1;
volatile uint8_t ch_request = CH1;
volatile uint8_t gain_select = 0;
volatile uint16_t dev_select = ADAR0;
volatile uint16_t adar_addr = ADAR0;
volatile uint8_t msg_id = 0;
volatile uint8_t error_code = 0;
volatile uint8_t ack = 0;
volatile uint8_t tr_ctl = TR_SPI_CTL;

volatile uint8_t data_type = direct;
volatile uint8_t amp_phase_mode = AMP_MODE;
volatile uint8_t rf_address;

uint8_t slave_address, row_1_pos, col_pos;
uint8_t row_alignment;
volatile uint8_t phase_data;
const uint8_t num_states = 128;
const uint8_t num_rows = 16;
float col_setting, row_setting;
uint8_t i=0;

//uint32_t transfer_data[] = {0x10, 0xA2, 0xC4, 0x25, 0xFE, 0x68, 0x42, 0xEA, 0x27, 0x45};
volatile uint32_t kx_data = 0, ky_data = 0;		// 32-bit data, will be converted to float after receiving
volatile uint32_t KxKy_temp = 0;
float kx_conv = 0;
float ky_conv = 0;

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	startup();

	// determine initial row position, column position, and whether lowest row is TX or RX (row_alignment)
	slave_address = get_slave_pos();
	row_1_pos = get_row(slave_address);
	col_pos = get_col(slave_address);
	if(ADDR0 == 0)
		row_alignment = ODD;	// For even addresses, the row alignment is "odd" (column number)
	else
		row_alignment = EVEN;	// For odd addresses, the row alignment is "even" (column number)

	//__bis_SR_register(GIE);	  // Enter LPM0 w/ interrupt

	//WDTCTL = WDTPW + WDTCNTCL;  // Kick the dog

	if (P2IN & TnR_IN)
	{
		TR_PORT |= ADAR_TnR;  // Put ADARs into Tx state
		tr_mode = TX;
	} else {
		TR_PORT &= ~ADAR_TnR;  // Put ADARs into Rx state
		tr_mode = RX;
	}

	power_enable(tr_mode);  // 8V powerup sequence

	// force TTL control
	spi_TR_mode(tr_mode, 1, ADAR0);
	spi_TR_mode(tr_mode, 1, ADAR2);

	while(1)
	{
		//WDTCTL = WDTPW + WDTCNTCL;  // Kick the dog
		ADDR_LEDs = 0;

		while (P2IN & CS_IN);  // sit here until CS goes low to initiate SPI

		// dump any trash in the buffer
		uint8_t trash = UCA0RXBUF;
		UCA0IFG &= ~UCRXIFG;  		// clear RX flag

		ADDR_LEDs |= ADDR1_LED;
		decode_SPI();
		ADDR_LEDs &= ~ADDR1_LED;

		// send the latch command to enable settings
		spi_load_working_reg(tr_mode, ADAR0);
		spi_load_working_reg(tr_mode, ADAR2);

		switch(tr_mode)
		{
			case TX:
				// load working registers
				TR_PORT |= TX_LOAD;
				//__delay_cycles(10);
				TR_PORT &= ~TX_LOAD;
				TR_PORT |= ADAR_TnR;  // Put ADARs into Tx state
				switch_power(TX);
			break;

			case RX:
				// load working registers
				TR_PORT |= RX_LOAD;
				//__delay_cycles(10);
				TR_PORT &= ~RX_LOAD;
				TR_PORT &= ~ADAR_TnR;  // Put ADARs into Rx state
				switch_power(RX);
			break;

			case TRX:
				// load working registers
				TR_PORT |= TX_LOAD;
				TR_PORT |= RX_LOAD;

				//__delay_cycles(10);
				TR_PORT &= ~TX_LOAD;
				TR_PORT &= ~RX_LOAD;

				TR_PORT |= ADAR_TnR;  // Put ADARs into Tx state
				switch_power(TX);
			break;
		}
	}
}

void set_mode_RS485(int mode) {
	if (mode == RXMODE485) {
		// RX Reciever is active low so set the bit low
		EN_TX_RX_PORT &= ~RX_EN_PIN;
		// TX Driver is active high so set bit low
		EN_TX_RX_PORT &= ~TX_EN_PIN;
	}
	else if (mode == TXMODE485) {
		// RX Reciever is active low so setthe bit high
		EN_TX_RX_PORT |= RX_EN_PIN;
		// TX Driver is active high so set bit high
		EN_TX_RX_PORT |= TX_EN_PIN;
	}
}

void PhaseState(float kx, float ky)
{
	col_setting = (kx*col_pos);

	switch(row_alignment)
	{
		case ODD: // lowest cell is TX
			for (ch_select = 0; ch_select < num_rows; ch_select = ch_select+2)
			{
				if (ch_select < (num_rows/2))  // set first ADAR phase
				{
					int i = ch_select;
					// calculate and set TX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(TX, ODD_ADDRESS_CH[ch_select], phase_data, ADAR0);

					i++; // increment row number, but keep the channel the same (TX row, RX row, same channel)
					// calculate and set RX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(RX, ODD_ADDRESS_CH[ch_select], phase_data, ADAR0);

				} else {						// set the second ADAR phase
					int i = ch_select;
					// calculate and set TX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(TX, ODD_ADDRESS_CH[ch_select], phase_data, ADAR2);

					i++; // increment row number, but keep the channel the same (TX row, RX row, same channel)
					// calculate and set RX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(RX, ODD_ADDRESS_CH[ch_select], phase_data, ADAR2);
				}
			}
		break;

		case EVEN: // lowest cell is RX
			for (ch_select = 0; ch_select < num_rows; ch_select = ch_select+2)
			{
				if (ch_select < (num_rows/2))  // set first ADAR phase
				{
					int i = ch_select;
					// calculate and set TX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(RX, EVEN_ADDRESS_CH[ch_select], phase_data, ADAR2);

					i++; // increment row number, but keep the channel the same (TX row, RX row, same channel)
					// calculate and set RX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(TX, EVEN_ADDRESS_CH[ch_select], phase_data, ADAR2);

				} else {
					int i = ch_select;
					// calculate and set TX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(RX, EVEN_ADDRESS_CH[ch_select], phase_data, ADAR0);

					i++; // increment row number, but keep the channel the same (TX row, RX row, same channel)
					// calculate and set RX phase
					row_setting = ky*(row_1_pos + i);
					phase_data = ((int)(row_setting + col_setting))&(num_states - 1);
					set_phase_by_channel(TX, EVEN_ADDRESS_CH[ch_select], phase_data, ADAR0);
				}
			}
		break;
	}
}

void clock_test()
{
	while(1)
	{
		// Clock speed check
		P1OUT |= BIT0;
		//__delay_cycles(10000);
		P1OUT &= ~BIT0;
		//__delay_cycles(10000);
	}
}

void set_ADARs(uint8_t tr_mode, uint8_t rx_amp_data, uint8_t tx_amp_data, uint8_t rx_phase_data, uint8_t tx_phase_data)
{
	for(ch_select = 1; ch_select <5; ch_select = ch_select + 1)
	{
		// set all channels phase or amplitude
		if(tr_mode == RX)
		{
			adar_reg_write(CH1_RX_GAIN + ch_select - 1, rx_amp_data, ADAR0);
			adar_reg_write(CH1_RX_GAIN + ch_select - 1, rx_amp_data, ADAR2);
			set_phase_by_channel(tr_mode, ch_select, rx_phase_data, ADAR0);
			set_phase_by_channel(tr_mode, ch_select, rx_phase_data, ADAR2);
//				adar_reg_read(CH1_RX_GAIN + ch_select - 1, ADAR0);
//				adar_reg_read(CH1_RX_GAIN + ch_select - 1, ADAR2);
		}
		else if(tr_mode == TX)
		{
			adar_reg_write(CH1_TX_GAIN + ch_select - 1, tx_amp_data, ADAR0);
			adar_reg_write(CH1_TX_GAIN + ch_select - 1, tx_amp_data, ADAR2);
			set_phase_by_channel(tr_mode, ch_select, tx_phase_data, ADAR0);
			set_phase_by_channel(tr_mode, ch_select, tx_phase_data, ADAR2);
//				adar_reg_read(CH1_TX_GAIN + ch_select - 1, ADAR0);
//				adar_reg_read(CH1_TX_GAIN + ch_select - 1, ADAR2);
		}
	}
}

void decode_SPI()
{
	while (!(P2IN & CS_IN))
	{
		//WDTCTL = WDTPW + WDTCNTCL;  // Kick the dog

		// wait for RX flag.  Check for CS to release while waiting to signify the start of a reset command
		int i = 0;
		while (!(WAIT_RX_FLAG))  // Wait for a SPI transmission
		{
			if ((!(P2IN & CS_IN)) && (i <= 3))  // if CS goes high, start counting cycles
			{
				while (!(P2IN & CS_IN))
				{
					ADDR_LEDs |= ADDR1_LED;
					if(WAIT_RX_FLAG)
						break;
				}

				while(P2IN & CS_IN)
				{
                                        ADDR_LEDs &= ~ADDR1_LED;
					if(WAIT_RX_FLAG)
						break;
				}
				i++;
			}
			// if count reached 3, assume a reset has been initiated
			if (i >= 3)
			{
				ADDR_LEDs |= ADDR0_LED;
				// reset SPI peripheral
				UCA0CTLW0 |= UCSWRST;					 // **Put state machine in reset**
				mode_byte_count = 0;		// reset byte count and mode and jump out
				msg_mode = HDR_SPI;
				UCA0CTLW0 &= ~UCSWRST;				// **Initialize USCI state machine**
				break;
			}
		}


		while(WAIT_RX_SPI); 		// wait for RX flag
		uint32_t c = UCA0RXBUF;		// read buffer

		 // reset comms sent, reset everything and jump out
		if ((c && 0x00FF) == RESET_COMMS)
		{
			UCA0IFG &= ~UCRXIFG;  		// clear RX flag
			mode_byte_count = 0;
			msg_mode = HDR_SPI;
			break;
		}

		while(WAIT_SPI);
		SPI_TX_BUFF = c & 0xFF;	// echo buffer

		UCA0IFG &= ~UCRXIFG;  		// clear RX flag

		switch(msg_mode) {
			case HDR_SPI:
				switch(mode_byte_count) {
				// first header byte
				case 0:
					// if bit 7 is set, reset or initialize the requested RF address and exit
					if (c & CONFIG_BIT)
					{
						if ((c & ALL_RST_INIT) | (c & 0x0F == slave_address & 0x0F))
						{
							// if it is reset, do the reset and skip to CRC
							if (c & RST_BIT)
							{
								reset_ADAR1000(ADAR0);		// reset ADARs
								reset_ADAR1000(ADAR2);
								mode_byte_count = 0;
								msg_mode = CRC_SPI;
								while (!(P2IN & CS_IN));	// loop for as long as CS is low
								break;
							}
							// initialize both devices and continue to CRC
							if(c & INIT_BIT)
							{
								init_ADAR1000(ADAR0);		// initialize ADARs
								init_ADAR1000(ADAR2);
								mode_byte_count = 0;
								msg_mode = CRC_SPI;
								while (!(P2IN & CS_IN)); 	// loop for as long as CS is low
								break;
							}
						}
						// if slave address is not requested, exit
						else break;
					}
					// if bit7 is not set, prepare for amplitude/phase commands
					else
					{
						if(c & CONST_BIT)  // constant amplitude or phase data coming
						{
							data_type = direct;
							if (c & AMP_nPHASE_BIT)
								amp_phase_mode = AMP_MODE;
							else
								amp_phase_mode = PHASE_MODE;
						} else
						{
							data_type = KxKy;
						}

						rf_address = (c & ADDR_BITS);
					}

					// otherwise move on to next header
					mode_byte_count++;
					if(mode_byte_count >= HDR_SIZE) {
						mode_byte_count = 0;
						msg_mode = DATA_SPI;
					}
				break;
				// second header byte
				case 1:

					tr_mode = ((c & TR_BITS) >> 6);

					ch_request = (c & CH_BITS) >> 2;

					// declare which ADAR is chosen and determine which channel should be set

					// TODO: set specified current gain
					gain_select = (c & GAIN_BITS);

					// count the byte in this mode and move on
					mode_byte_count++;
					if(mode_byte_count >= HDR_SIZE)
										{
						mode_byte_count = 0;
						msg_mode = DATA_SPI;
					}
				break;
				}
			break;

			case DATA_SPI:
				switch(data_type)
				{
					case direct: // constant amplitude or phase, can be sent to one, many, or all channels
						switch(row_alignment)
						{
							case EVEN:
								if (ch_request == (CH1 | CH2 | CH3 | CH4))
								{
									dev_select = ADAR0;
									ch_select = EVEN_ADDRESS_CH[ch_request];
									ch_range = 1;
								} else if (ch_request == (CH5 | CH6 | CH7 | CH8))
								{
									dev_select = ADAR2;
									ch_select = EVEN_ADDRESS_CH[ch_request];
									ch_range = 1;
								} else if (ch_request == CH1_4)
								{
									dev_select = ADAR0;
									ch_select = EVEN_ADDRESS_CH[CH1];
									ch_range = 4;
								} else if (ch_request == CH5_8)
								{
									dev_select = ADAR2;
									ch_select = EVEN_ADDRESS_CH[CH5];
									ch_range = 4;
								} else if (ch_request == ALL_CH)
								{
									dev_select = ALL_ADAR;
									ch_select = EVEN_ADDRESS_CH[CH1];
									ch_range = 4;
								}
							break;

							case ODD:
								if (ch_request == (CH1 | CH2 | CH3 | CH4))
								{
									dev_select = ADAR0;
									ch_select = ODD_ADDRESS_CH[ch_request];
									ch_range = 1;
								} else if (ch_request == (CH5 | CH6 | CH7 | CH8))
								{
									dev_select = ADAR2;
									ch_select = ODD_ADDRESS_CH[ch_request];
									ch_range = 1;
								} else if (ch_request == CH1_4)
								{
									dev_select = ADAR0;
									ch_select = ODD_ADDRESS_CH[CH1];
									ch_range = 4;
								} else if (ch_request == CH5_8)
								{
									dev_select = ADAR2;
									ch_select = ODD_ADDRESS_CH[CH5];
									ch_range = 4;
								} else if (ch_request == ALL_CH)
								{
									dev_select = ALL_ADAR;
									ch_select = ODD_ADDRESS_CH[CH1];
									ch_range = 4;
								}
							break;
						}

						// write to one or many channels
						switch(ch_range)
						{
							case 1:
								// set the corresponding channel phase or amplitude
								if(amp_phase_mode == AMP_MODE)
								{
									if(tr_mode == RX)
									{
										adar_reg_write(CH1_RX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
									}
									else if(tr_mode == TX)
									{
										adar_reg_write(CH1_TX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
									}
									else if(tr_mode == TRX)
									{
										adar_reg_write(CH1_RX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
										adar_reg_write(CH1_TX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
									}
								}
								else if(amp_phase_mode == PHASE_MODE)
								{
									if (tr_mode == TRX)
									{
										set_phase_by_channel(TX, ch_select, c & 0x7F, dev_select);
										set_phase_by_channel(RX, ch_select, c & 0x7F, dev_select);
									}
									else
									{
										set_phase_by_channel(tr_mode, ch_select, c & 0x7F, dev_select);
									}
								}
								while (!(P2IN & CS_IN)); // loop until master releases
								// count the byte in this mode and move on
								mode_byte_count++;
								if(mode_byte_count >= DATA_SIZE)
								{
									mode_byte_count = 0;
									msg_mode = CRC_SPI;
								}
							break;

							// if all channels are selected, cycle through them and set
							case 4:
								for(ch_select = 1; ch_select <5; ch_select = ch_select + 1)
								{
									if(amp_phase_mode == AMP_MODE)
									{
										if(tr_mode == RX)
										{
											adar_reg_write(CH1_RX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
										}
										else if(tr_mode == TX)
										{
											adar_reg_write(CH1_TX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
										}
										else if(tr_mode == TRX)
										{
											adar_reg_write(CH1_RX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
											adar_reg_write(CH1_TX_GAIN + ch_select - 1, BIT7 | (c & 0x7F), dev_select);
										}
									}
									else if(amp_phase_mode == PHASE_MODE)
									{
										if (tr_mode == TRX)
										{
											set_phase_by_channel(TX, ch_select, c & 0x7F, dev_select);
											set_phase_by_channel(RX, ch_select, c & 0x7F, dev_select);
										}
										else
										{
											set_phase_by_channel(tr_mode, ch_select, c & 0x7F, dev_select);
										}
									}
								}
								while (!(P2IN & CS_IN)); // loop until master releases, if needed
								// count the byte in this mode and move on
								mode_byte_count++;
								if(mode_byte_count >= DATA_SIZE)
								{
									mode_byte_count = 0;
									msg_mode = CRC_SPI;
								}
							break;
						}
					break;

					case KxKy:
						switch(mode_byte_count)
						{
							case 0:  // Kx MSB
								// ensure variables are clear when Kx and Ky are initiated
								kx_data = 0;
								KxKy_temp = 0;
								kx_conv = 0;
								kx_data = (c & 0x000000FF) << 24;
								mode_byte_count++;
							break;

							case 1:  // Kx byte 2
								KxKy_temp = (c & 0x000000FF) << 16;
								kx_data = kx_data | KxKy_temp;
								mode_byte_count++;
							break;

							case 2:  // Kx byte 1
								KxKy_temp = (c & 0x000000FF) << 8;
								kx_data = kx_data | KxKy_temp;
								mode_byte_count++;
							break;

							case 3:  // Kx LSB
								KxKy_temp = (c & 0x000000FF);
								kx_data = kx_data | KxKy_temp;
								kx_conv = *(float *) &kx_data;  // long int complete, convert to float
								mode_byte_count++;
							break;

							case 4:  // Ky MSB
								// ensure variables are clear when Kx and Ky are initiated
								ky_data = 0;
								KxKy_temp = 0;
								ky_conv = 0;
								ky_data = (c & 0x000000FF) << 24;
								mode_byte_count++;
							break;

							case 5:  // Ky byte 2
								KxKy_temp = (c & 0x000000FF) << 16;
								ky_data = ky_data | KxKy_temp;
								mode_byte_count++;
							break;

							case 6:  // Ky byte 1
								KxKy_temp = (c & 0x000000FF) << 8;
								ky_data = ky_data | KxKy_temp;
								mode_byte_count++;
							break;

							case 7:  // Ky LSB
								KxKy_temp = (c & 0x000000FF);
								ky_data = ky_data | KxKy_temp;
								ky_conv = *(float *) &ky_data;  // long int complete, convert to float
								mode_byte_count++;
								if(mode_byte_count >= KXKY_SIZE)
								{
									while (!(P2IN & CS_IN)); // loop until master releases
									// all bytes are in (pending CRC), calculate phase data and load registers
									PhaseState(kx_conv,ky_conv);
									mode_byte_count = 0;
									msg_mode = CRC_SPI;

									//ADDR_LEDs |= ADDR3_LED;
									//spi_writeKxKy(kx_conv,ky_conv);
									//ADDR_LEDs &= ~ADDR3_LED;
								}
							break;
						}
					break;
				}
			break;

			case CRC_SPI:
				//__delay_cycles(1);
				configure_uart();
				set_mode_RS485(TXMODE485);
				//__delay_cycles(100);
				// send back acknowledge
				//UCA1TXBUF = msg_id;
				while(WAIT_SPI);
				// count the byte in this mode and move on
				mode_byte_count++;
				if(mode_byte_count >= CRC_SIZE)
								{
					mode_byte_count = 0;
					msg_mode = HDR_SPI;
				}
				ack = 0;
			break;
		}
	}
	// if CS goes high before all transfers complete, reset transfer status
	mode_byte_count = 0;
	msg_mode = HDR_SPI;
}

void spi_writeKxKy(float kx, float ky)
{
	ADDR_LEDs |= ADDR3_LED;  // set LED3 to signal echo
	uint8_t KxKy_data_out[] = {0,0,0,0,0,0,0,0,0,0};  				// ten bytes of data required for Kx Ky setting
	uint8_t num_bytes = 10;

	signed long kx_data = *(signed long *)&kx;
	signed long ky_data = *(signed long *)&ky;

	// build data structure
	KxKy_data_out[0] = 0x55;
	KxKy_data_out[1] = 0xAA;

	// 32 bits of Kx data, 4 bytes
	KxKy_data_out[2] = (uint8_t)((kx_data >> 24) & (0xFF));
	KxKy_data_out[3] = (uint8_t)((kx_data >> 16) & (0xFF));
	KxKy_data_out[4] = (uint8_t)((kx_data >> 8) & (0xFF));
	KxKy_data_out[5] = (uint8_t)((kx_data) & (0xFF));

	// 32 bits of Kx data, 4 bytes
	KxKy_data_out[6] = (uint8_t)((ky_data >> 24) & (0xFF));
	KxKy_data_out[7] = (uint8_t)((ky_data >> 16) & (0xFF));
	KxKy_data_out[8] = (uint8_t)((ky_data >> 8) & (0xFF));
	KxKy_data_out[9] = (uint8_t)((ky_data) & (0xFF));

	int i = 0;

	do
	{
		// wait for clear SPI buffer
		while(WAIT_SPI);
		SPI_TX_BUFF = KxKy_data_out[i];
		i++;
	}while (i<num_bytes);

	ADDR_LEDs &= ~ADDR3_LED;  // set LED3 to signal echo

	return;
}
