//---------------------------------------------------------------------------
// Copyright (C) 2001 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//---------------------------------------------------------------------------

/*!
 * @addtogroup DS28E05
 * @brief This layer provides functions for supporting 1-wire EEPROM read/writes
 * @{*/


#include <stdbool.h>
#include <nrf.h>
#include <nrf_gpio.h>

#include "ds28e05.h"

#define TICK_PER_US 					UINT8_C(16)
#define OW_PIN 							UINT8_C(15)

#define tSLOT    						UINT8_C(13)
#define tRSTL   						UINT8_C(60)
#define tRSTH    						UINT8_C(48)
#define tMSP     						UINT8_C(10)

#define tWOL     						UINT8_C(10)
#define tW1L    						UINT8_C(1)
#define tRL      						UINT8_C(1)
#define tMSR     						UINT8_C(2)

NRF_GPIO_Type *nrf_port_reg;
/**
 * Global variable declarations
 */
static volatile bool running = false;
static int volatile sample = 0;

uint8_t crc88540_table[256] = {
		0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,\
		157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,\
		35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,\
		190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,\
		70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,\
		219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,\
		101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,\
		248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,\
		140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,\
		17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,\
		175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,\
		50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,\
		202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,\
		87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,\
		233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,\
		116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

/*
 * Static function declarations
 */
static bool ds28e05_touch_bit(uint8_t);
static uint8_t ds28e05_touchbyte(uint8_t);
static uint8_t ds28e05_get_crc(uint8_t *data, uint8_t count);

static uint8_t ds28e05_get_crc(uint8_t *data, uint8_t count)
{
	uint8_t result=0;

	while(count--) {
		result = crc88540_table[result ^ *data++];
	}

	return result;
}

uint8_t read_buff[20] = {0};

void ds28e05_Init(void)
{
	if(EEPROM_PORT == 0)
	{
		nrf_port_reg = NRF_P0;
	}
	else
	{
		nrf_port_reg = NRF_P1;
	}
	nrf_port_reg->PIN_CNF[EEPROM_PORT_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
	                                    		  | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
												  | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
												  | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
												  | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	nrf_port_reg->OUTSET = (1 << EEPROM_PORT_PIN);

	/**
	 * CC0 sets OWPIN to 0
	 * CC1 sets OWPIN to 1
	 * CC2 Sampling of the GPIO
	 * CC3 stops the timer and clear a flag (interrupt routine defined in owlnk.c)
	 */

	NRF_TIMER1->PRESCALER = 0;

	/* Enable Interrupt for compare 2 and 3 */
	NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk | TIMER_INTENSET_COMPARE2_Msk | TIMER_INTENSET_COMPARE3_Msk;
	NRF_TIMER1->BITMODE = 3;
	NVIC_SetPriority(TIMER1_IRQn, 1);
	NVIC_EnableIRQ(TIMER1_IRQn);

	/* Compare 3 shorts to stop */
	NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE3_STOP_Msk | TIMER_SHORTS_COMPARE3_CLEAR_Msk;
}

void TIMER1_IRQHandler(void)
{
	if (NRF_TIMER1->EVENTS_COMPARE[0]) {
		nrf_port_reg->OUTCLR = (1 << EEPROM_PORT_PIN);
		NRF_TIMER1->EVENTS_COMPARE[0] = 0;
	}

	if (NRF_TIMER1->EVENTS_COMPARE[1]) {
		nrf_port_reg->OUTSET = (1 << EEPROM_PORT_PIN);
		NRF_TIMER1->EVENTS_COMPARE[1] = 0;
	}

	if (NRF_TIMER1->EVENTS_COMPARE[2]) {
		sample = (nrf_port_reg->IN >> EEPROM_PORT_PIN)&0x01;
		NRF_TIMER1->EVENTS_COMPARE[2] = 0;
	}

	if (NRF_TIMER1->EVENTS_COMPARE[3]) {
		running = false;
		NRF_TIMER1->EVENTS_COMPARE[3] = 0;
	}
}


/**
 * Reset the ds28e05 EEPROM devices on the 1-Wire Net and return the result.
 *
 * Returns: TRUE(1):  presence pulse(s) detected, device(s) reset
 *          FALSE(0): no presence pulses detected
 */
bool ds28e05_reset(void)
{
	sample = 1;

	NRF_TIMER1->TASKS_CLEAR = 1;
	NRF_TIMER1->CC[0] = 1;
	NRF_TIMER1->CC[1] = 60*TICK_PER_US;
	NRF_TIMER1->CC[2] = (60+9)*TICK_PER_US;
	NRF_TIMER1->CC[3] = (60+60)*TICK_PER_US;;

	running = true;
	NRF_TIMER1->TASKS_START = 1;

	while(running);

	if (sample == 0)
		return 1;

	return 0;
}

/**
 * Send 1 bit of communication to the 1-Wire Net and return the
 * result 1 bit read from the 1-Wire Net.  The parameter 'sendbit'
 * least significant bit is used and the least significant bit
 * of the result is the return bit.
 *
 * Returns: 0:   0 bit read from sendbit
 * 			1:   1 bit read from sendbit
 */
static bool ds28e05_touch_bit(uint8_t sendbit)
{
	NRF_TIMER1->CC[0] = 1;
	NRF_TIMER1->CC[1] = (sendbit&0x01)?(1*TICK_PER_US)+1:10*TICK_PER_US;
	NRF_TIMER1->CC[2] = (2+1)*TICK_PER_US;
	NRF_TIMER1->CC[3] = (10+10)*TICK_PER_US;

	running = true;
	NRF_TIMER1->TASKS_START = 1;

	while(running);

	return sample;
}

/**
 * Send 8 bits of communication to the 1-Wire Net and return the
 * result 8 bits read from the 1-Wire Net.  The parameter 'sendbyte'
 * least significant 8 bits are used and the least significant 8 bits
 * of the result is the return byte.
 *
 * 'sendbyte'   - 8 bits to send (least significant byte)
 *
 * Return:  	- 8 bytes read from sendbyte
 */
static uint8_t ds28e05_touchbyte(uint8_t sendbyte)
{
	uint8_t i;
	uint8_t receivedbyte=0;

	for (i=0; i<8; i++)
	{
		receivedbyte >>= 1;
		receivedbyte |= ds28e05_touch_bit(sendbyte)?0x80:0;
		sendbyte >>= 1;
	}

	return receivedbyte;
}

/*
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.
 *
 * 'sendbyte'   - 8 bits to send (least significant byte)
 *
 * Returns:	TRUE: bytes written and echo was the same
 *			FALSE: echo was not the same
 */
bool ds28e05_writebyte(uint8_t sendbyte)
{
	return (ds28e05_touchbyte(sendbyte) == sendbyte) ? true : false;
}

/*
 *  Send 8 bits of read communication to the 1-Wire Net and and return the
 *  result 8 bits read from the 1-Wire Net.
 *
 *  Returns:  8 bytes read from 1-Wire Net
 */
uint8_t ds28e05_readbyte(void)
{
	return ds28e05_touchbyte(0xFF);
}

/**
 * Provides delay in ms
 * 'len'	- number of milliseconds
 *
 */
void ds28e05_msDelay(uint16_t len)
{
	NRF_TIMER1->TASKS_CLEAR = 1;
	NRF_TIMER1->CC[0] = 0;
	NRF_TIMER1->CC[1] = 0;
	NRF_TIMER1->CC[2] = 0;
	NRF_TIMER1->CC[3] = (len*TICK_PER_US)*1000;
	running = true;
	NRF_TIMER1->TASKS_START = 1;

	while(running);
}

/**
 * Read the internal rom id, and checks whether the read id is correct or not
 *
 * Returns : 	true  : read the correct rom id
 * 				false : unable to read the rom id or slave is not responding
 */
bool ds28e05_read_rom_id(uint8_t *buffer)
{
	uint8_t i = 0;
	uint8_t crc = 0;
	if(!ds28e05_reset())
		return false;
    /** return of write byte is not validated as it returning failure always. This will be analyzed later */
    ds28e05_writebyte(DS28E05_READ_ROM_CMD);
	for(i = 0 ; i < DS28E05_ROM_ID_SIZE ; i++)
		buffer[i] = ds28e05_readbyte();

	crc = ds28e05_get_crc(buffer,7);
	if(crc == buffer[7])
	{
		return true;
	}
	else
	{
		return false;
	}
}
