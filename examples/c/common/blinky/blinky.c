/**\
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 * BSD-3-Clause
 * Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 * Works only for MCU_NICLA, MCU_APP30 and MCU_APP31 targets
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <stdlib.h>
#include "coines.h"
#include "led.h"
/******************************************************************************/
/*!                Macro definition                                           */

/******************************************************************************/
/*!           Static Function Declaration                                     */

/******************************************************************************/
/*!            Functions                                        */
/******************************************************************************/

extern struct led_dev led_dev; 

/* This function starts the execution of program. */
int main(void)
{
	struct coines_comm_intf_config intfconfig =
	{
			.uart_baud_rate = COINES_UART_BAUD_RATE_9600
	};

	/* Initializes COINES_SDK platform */
	int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB, &intfconfig);
	if (result < COINES_SUCCESS)
	{
		//printf(
		//    "\n Unable to connect with NICLA Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
        //    " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(result);
    }

    while (1)
    {

        result = coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
        //led_setcolorandintensity(&led_dev, BLUE, 0x80);	//Color:blue ; Output current = intensity/256 * Imax[set to 30mA] 
        coines_delay_msec(1000);

        //led_setcolorandintensity(&led_dev, BLUE, 0x00);	//Color:blue ; Output current = intensity/256 * Imax[set to 30mA] 
        result = coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
        coines_delay_msec(1000);
    }

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    return result;
}
