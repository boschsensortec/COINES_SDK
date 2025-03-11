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

 * Works only for MCU_NICLA target
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <stdlib.h>
#include "coines.h"
#include "bq25120.h"
/******************************************************************************/
/*!                Macro definition                                           */

/******************************************************************************/
/*!           Static Function Declaration                                     */
/*! Callback for button 1 interrupt */
//static void button1CBNicla(uint32_t param1, uint32_t param2);
/******************************************************************************/
/*!            Functions                                        */
/******************************************************************************/

extern struct bq_dev pmic_dev;
/* This function starts the execution of program. */
int main(void)
{
    struct bq_status bqstatus = {0, 0, 0, 0, 0, 0};
    struct fault_mask_reg faults = {0, 0, 0, 0};
    struct ts_flt_mask_config tscfg = {0, 0, 0, 0, 0};

	struct coines_comm_intf_config intfconfig =
	{
			.uart_baud_rate = COINES_UART_BAUD_RATE_9600
	};

	/* Initializes COINES_SDK platform */
	int16_t result = coines_open_comm_intf(COINES_COMM_INTF_BLE, &intfconfig);
	if (result < COINES_SUCCESS)
	{
		//printf(
		//    "\n Unable to connect with NICLA Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
        //    " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
        exit(result);
    }

    //LS LDO output configuration
    coines_set_shuttleboard_vdd_vddio_config(0, 1800); //Set lsldo output to 1,8V 

    //TS control and mask fault configuration
    tscfg.ts_fct = BQ_TS_FCT_DISABLE;
    tscfg.int_fct = BQ_INT_FCT_ENABLE;
    tscfg.mr_wake = BQ_MR_WAKE_IT_ENABLE;
    tscfg.mr_reset = BQ_MR_RESET_IT_ENABLE;
	tscfg.timer_fault = BQ_TIMER_FAULT_IT_ENABLE;
    bq_set_ts_ctrl_fault_mask(&pmic_dev, tscfg);
    
    while (1)
    {
        //Get PMIC Status register
        bq_get_status(&pmic_dev, &bqstatus);
        printf("BQ status = %d\r\n",bqstatus.status);
        printf("BQ CD line = %d\r\n",bqstatus.cd_status);

        switch (bqstatus.status)
        {
        case 0: /*READY*/
            coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
            coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
            coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
            break;
        case 1:/*Charge in progress*/
            coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
            break;
        case 2:/*Charge done*/
            coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
            break;
        case 3:/*Fault*/
            coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
            break;
        default:
            break;
        }
        bqstatus.status = 0 ;

        //Get PMIC faults
        bq_get_faults(&pmic_dev, &faults);
        printf("BQ fault VIN OV = %d\r\n",faults.vin_ov);
        printf("BQ fault VIN UV = %d\r\n",faults.vin_uv);
        printf("BQ fault BAT UVLO = %d\r\n",faults.bat_uvlo);
        printf("BQ fault BAT OCP = %d\r\n",faults.bat_ocp);

        //Get battery voltage
        printf("BQ Battery level in percentage = %d %% \r\n", pmic_pull_battery_level());

        coines_delay_msec(2000);

        coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
        coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
        coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);

        coines_delay_msec(1000);
    }

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    return result;
}
