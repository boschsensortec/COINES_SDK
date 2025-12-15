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

 * Works only for MCU_APP31 target
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <stdlib.h>
#include "coines.h"
#include "bq25120.h"

extern struct bq_dev pmic_dev;
extern bool ble_nus_connected;
extern bool serial_connected;

/* This function starts the execution of program. */
int main(void)
{
    struct bq_status bqstatus = { 0, 0, 0, 0, 0, 0 };
    struct fault_mask_reg faults = { 0, 0, 0, 0 };
    uint16_t bat_status_mv = 0;
    uint8_t bat_status_percent = 0;

    struct coines_ble_config bleconfig = {
        .name = "PMIC_Test", .tx_power = COINES_TX_POWER_8_DBM
    };

    coines_ble_config(&bleconfig);

    /* Initializes COINES_SDK platform */
    int16_t result = coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);

    if (result < COINES_SUCCESS)
    {
        /*
         * printf(
         *    "\n Unable to connect with NICLA Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2.
         * Check if Application Board USB driver is installed. \n"
         *    " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
         */
        exit(result);
    }

    while (1)
    {
        /*Get PMIC Status register */
        bq_get_status(&pmic_dev, &bqstatus);
        coines_delay_msec(100);
        
        /*Get PMIC faults */
        bq_get_faults(&pmic_dev, &faults);
        coines_delay_msec(100);
        
        /*Get battery status */
        (void)coines_read_bat_status(&bat_status_mv, &bat_status_percent);

        if (ble_nus_connected)
        {
            fprintf(bt_w, "BQ status = %d\r\n", bqstatus.status);
            fprintf(bt_w, "BQ CD line = %d\r\n", bqstatus.cd_status);
            fprintf(bt_w, "BQ fault = \r\n");
            fprintf(bt_w, "BQ fault VIN OV = %d\r\n", faults.vin_ov);
            fprintf(bt_w, "BQ fault VIN UV = %d\r\n", faults.vin_uv);
            fprintf(bt_w, "BQ fault BAT UVLO = %d\r\n", faults.bat_uvlo);
            fprintf(bt_w, "BQ fault BAT OCP = %d\r\n", faults.bat_ocp);
            fprintf(bt_w, "BQ Battery level in percentage = %d %% \r\n", bat_status_percent);
            fprintf(bt_w, "BQ Battery voltage in mV = %d mV\r\n", bat_status_mv);
        }
        else // serial_connected
        {
            printf("BQ status = %d\r\n", bqstatus.status);
            printf("BQ CD line = %d\r\n", bqstatus.cd_status);
            printf("BQ fault = \r\n");
            printf("BQ fault VIN OV = %d\r\n", faults.vin_ov);
            printf("BQ fault VIN UV = %d\r\n", faults.vin_uv);
            printf("BQ fault BAT UVLO = %d\r\n", faults.bat_uvlo);
            printf("BQ fault BAT OCP = %d\r\n", faults.bat_ocp);
            printf("BQ Battery level in percentage = %d %% \r\n", bat_status_percent);
            printf("BQ Battery voltage in mV = %d mV\r\n", bat_status_mv);
        }

        coines_delay_msec(5000);
    }

    coines_close_comm_intf(COINES_COMM_INTF_BLE, NULL);

    return result;
}
