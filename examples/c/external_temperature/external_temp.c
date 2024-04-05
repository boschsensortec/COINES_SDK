/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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