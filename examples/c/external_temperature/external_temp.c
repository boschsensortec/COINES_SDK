/**\
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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

 * Works for MCU_APP30 and MCU_APP31 targets
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "ext_temperature.h"

/******************************************************************************/
/*!                Macro definition                                           */

/******************************************************************************/
/*!           Static Function Declaration                                     */

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print accel data. */
    uint8_t limit = 20;

    /* Temperature sensor initialization configuration. */
    struct external_temp_dev temp_dev;

    /* Variable to hold temperature sensor data. */
    float temperature_data = 0;

    uint8_t indx = 1;

    /* Initialize COINES_SDK platform */
    rslt = external_temp_coines_init(COINES_COMM_INTF_USB);
    if(rslt != COINES_SUCCESS)
    {   
        external_temp_error_codes_print_result("external_temp_coines_init", rslt);
        return rslt;
    }

    /* Initialize external temperature sensor interface (I2C_1) */
    rslt = external_temp_interface_init(&temp_dev);
    if(rslt != COINES_SUCCESS)
    {   
        external_temp_error_codes_print_result("external_temp_interface_init", rslt);
        return rslt;
    }

    /* Read and print external temperature data. */
    for (;indx <= limit; indx++ )
    {
        printf("\n*******  External temperature iteration : %d  *******", indx);
        
        /* Get external temperature data. */
        rslt = external_temp_get_sensor_data(&temp_dev, &temperature_data);
        if(rslt != COINES_SUCCESS)
        {   
            external_temp_error_codes_print_result("external_temp_get_sensor_data", rslt);
            return rslt;
        }

        printf("\nTemperature data = %f", temperature_data);
    }

    /* De-initialize COINES_SDK platform */
    external_temp_coines_deinit(COINES_COMM_INTF_USB);

    return rslt;
}