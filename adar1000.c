/*
 * adar1000.c
 *
 *  Created on: Jul 23, 2019
 *      Author: R. Crismon
 *		Modified for MFP by J. Rhodes
 */

#include "adar1000.h"
#include "MSP_setup.h"

void select_device(uint16_t device)
{
    // set chip select low on desired device
	if (device == ADAR0)  // addr 00
		CS_PORT &= ~ADAR1_nEN;
	else if (device == ADAR1)  // addr 01
	    CS_PORT |= ADAR1_nEN | ADAR2_nEN;
	else if (device == ADAR2)  // addr 10
		CS_PORT &= ~ADAR2_nEN;
	else if (device == ADAR3)  // addr 11
	    CS_PORT |= ADAR1_nEN | ADAR2_nEN;
	else  // address all
	{
        CS_PORT &= ~ADAR1_nEN;
        CS_PORT &= ~ADAR2_nEN;
	}
}


uint8_t adar_reg_write(uint16_t address, uint8_t data, uint16_t device)
{
    // wait for clear SPI buffer
    while(WAIT_SPI);

    // enable the correct device SPI
	select_device(device);

	// send upper 8 bits of address
	SPI_TX_BUFF = (device | address) >> 8;
	// wait for clear SPI buffer

    // wait for clear SPI buffer
    while(WAIT_SPI);

    // send lower 8 bits of address
    SPI_TX_BUFF = (uint8_t) (address & 0xFF);

    // wait for clear SPI buffer
    while(WAIT_SPI);

    // send the data
    SPI_TX_BUFF = data;

    // wait for clear SPI buffer
    while(WAIT_SPI);

	// disable all SPI devices
	CS_PORT |= ADAR1_nEN | ADAR2_nEN;

    return 0;
}

uint8_t adar_reg_read(uint16_t address, uint16_t device) {

    // wait for clear SPI buffer
    while(WAIT_SPI);

    // enable the correct device SPI
	select_device(device);

    // send upper 8 bits of address
    SPI_TX_BUFF = (READ_ADDR | device | address) >> 8;

    // wait for clear SPI buffer
    while(WAIT_SPI);
    // send lower 8 bits of address
    SPI_TX_BUFF = address & 0xFF;

    // wait for clear SPI buffer
    while(WAIT_SPI);
    // send dummy bits for read
    SPI_TX_BUFF = 0x00;

    // wait for clear SPI buffer
    while(WAIT_SPI);
   // wait to receive register contents
    // while(WAIT_RX_FLAG);
    uint8_t rx = SPI_RX_BUFF;
	UCB1IFG &= ~UCRXIFG;  // clear RX flag

	// disable all SPI devices
	CS_PORT |= ADAR1_nEN | ADAR2_nEN;
    return rx;

}

uint8_t set_phase_by_channel(uint8_t tr, uint8_t channel, uint8_t phase, uint16_t device)
{

    // phase setting between 0 and 2^7 - 1 = 127
    if(phase > 127) return 1;
    // channel between 1 and 4
    else if(channel > 4 || channel < 1) return 2;
    // tr must be RX or TX
    else if ((tr != RX) && (tr != TX)) return 3;

    // get the correct registers to write to
    uint16_t addr_i = PHASE_ADDR_I[(tr << 2) + channel - 1];
    uint16_t addr_q = PHASE_ADDR_Q[(tr << 2) + channel - 1];

    // phase data for In-Phase
    uint8_t data_i = VI_PHASES[phase];
    // phase data for Quadrature
    uint8_t data_q = VQ_PHASES[phase];

    // write the VI Phase
    adar_reg_write(addr_i,data_i, device);
    // write the VQ Phase
    adar_reg_write(addr_q,data_q, device);

    return 0;
}

void init_ADAR1000(uint16_t device)
{
    // initialize the device
    reset_ADAR1000(device);
    // read back the LDO control register
    uint8_t j = adar_reg_read(0x0400, device);

    // enable spi control by bypassing memory control
    enable_spi_control(device);
    // read back memory control register that you just wrote to
    j = adar_reg_read(0x0038, device);
    // enable all RX channels and VM, VGA, LNA by writing 0x7F to reg 0x2E
    adar_reg_write(RX_ENABLES, ALL_CH_RX_EN | RX_LNA_EN | RX_VM_EN | RX_VGA_EN, device);
    // read back the register you just wrote to
    j = adar_reg_read(RX_ENABLES, device);

    // set up the bias currents for all of the RX channels
    adar_reg_write(BIAS_CURRENT_RX_LNA,0x08, device);
    adar_reg_write(BIAS_CURRENT_RX,0x16, device);
    // enable all RX
    adar_reg_write(SW_CTRL, RX_EN, device);
    // read back the switch control register
    j = adar_reg_read(SW_CTRL, device);

    // initialize RX Channels
    initialize_channel(1,RX, device);
    initialize_channel(2,RX, device);
    initialize_channel(3,RX, device);
    initialize_channel(4,RX, device);

    adar_reg_write(TX_ENABLES, ALL_CH_TX_EN | TX_DRV_EN | TX_VM_EN | TX_VGA_EN, device);
    // read back the register you just wrote to
//       j = adar_reg_read(TX_ENABLES);

    // set up the bias currents for all of the TX channels
    adar_reg_write(BIAS_CURRENT_TX_DRV,TX_DRV_BIAS_NOM, device);
    adar_reg_write(BIAS_CURRENT_TX,TX_VGA_BIAS_NOM|TX_VM_BIAS_NOM, device);
    // enable all TX
    adar_reg_write(SW_CTRL, TX_EN | TR_SPI, device);
    // read back the switch control register
    j = adar_reg_read(SW_CTRL, device);

    initialize_channel(4,TX, device);
    initialize_channel(3,TX, device);
    initialize_channel(2,TX, device);
    initialize_channel(1,TX, device);
}

void reset_ADAR1000(uint16_t device) {
    // soft reset the ADAR1000 with writing 0x81 to 0x0000
    adar_reg_write(INTERFACE_CONFIG_A,SOFTRESET, device);
    // enable MISO readback with writing 0x18 to 0x0000
    adar_reg_write(INTERFACE_CONFIG_A,SDOACTIVE, device);
    // trim the internal LDO according to example from ADI
    adar_reg_write(LDO_TRIM_CTL_0, 0x55, device);
}

void enable_spi_control(uint16_t device) {
    // enable spi control of phase and bias by writing 0x60 to 0x0038
    adar_reg_write(MEM_CTRL, BEAM_RAM_BYPASS | BIAS_RAM_BYPASS, device);
}

void spi_TR_mode(uint8_t mode, uint8_t spi_ctl, uint16_t device)
{
    if (mode == RX) {
        // enable all RX
        adar_reg_write(SW_CTRL, RX_EN | (spi_ctl << 2), device);
    }
    if (mode == TX) {
        // enable all TX
        adar_reg_write(SW_CTRL, TX_EN | TR_SPI | (spi_ctl << 2), device);
    }
}

void spi_load_working_reg(uint8_t mode, uint16_t device)
{
    switch(mode)
    {
		case RX:
			// Load the RX gain and phase settings to SPI
			adar_reg_write(LD_WRK_REGS,LDRX_OVERRIDE, device);
			adar_reg_write(LD_WRK_REGS,0x00, device);
		break;

		case TX:
			// Load the TX gain and phase settings to SPI
			adar_reg_write(LD_WRK_REGS,LDTX_OVERRIDE, device);
			adar_reg_write(LD_WRK_REGS,0x00, device);
		break;

		case TRX:
			// Load the RX gain and phase settings to SPI
			adar_reg_write(LD_WRK_REGS,LDRX_OVERRIDE, device);
			adar_reg_write(LD_WRK_REGS,0x00, device);

			// Load the TX gain and phase settings to SPI
			adar_reg_write(LD_WRK_REGS,LDTX_OVERRIDE, device);
			adar_reg_write(LD_WRK_REGS,0x00, device);
		break;
    }

}

uint8_t initialize_channel(uint8_t channel, uint8_t tr, uint16_t device)
{
    if(channel < 1 || channel > 4) {
        return 1;
    }
    if (tr > TX) {
        return 2;
    }

	switch(tr) {
	case RX:
		adar_reg_write(CH1_RX_GAIN + channel - 1, MAX_GAIN | ATTN_SETTING ,device);
		set_phase_by_channel(tr, channel, 0, device);
		// Load the RX gain and phase settings to SPI
		adar_reg_write(LD_WRK_REGS,LDRX_OVERRIDE, device);
		adar_reg_write(LD_WRK_REGS,0x00, device);
	break;
	case TX:
		adar_reg_write(CH1_TX_GAIN + channel - 1, MAX_GAIN | ATTN_SETTING, device);
		set_phase_by_channel(tr, channel, 0, device);
		// Load the TX gain and phase settings to SPI
		adar_reg_write(LD_WRK_REGS,LDTX_OVERRIDE, device);
		adar_reg_write(LD_WRK_REGS,0x00, device);
	break;
	}
	return 0;
}

void ttl_TR_mode(uint8_t mode) {
    if(mode > TX) return;
    // set high for TX and low for RX
    if (mode == TX) {
        TR_PORT |= ADAR_TnR;
    }
    if(mode == RX) {
        TR_PORT &= ~ADAR_TnR;
    }
}
