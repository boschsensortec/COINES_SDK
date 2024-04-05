/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
