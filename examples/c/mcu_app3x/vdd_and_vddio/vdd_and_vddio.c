/**
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
 *
 * @file    vdd_and_vddio.c
 * @brief   Test different vdd and vddio ranges.
 *          Works on MCU_APP31 target with the BMI270 sensor. For other sensors, 
 *          update the I2C address and voltage range based on the datasheet. 
 */

#include <stdio.h>
#include <stdbool.h>
#include "coines.h"

#define BMI2_I2C_PRIM_ADDR  0x68

int main(void)
{
    uint8_t chip_id;
    uint16_t no_of_bytes = 1;
	uint8_t reg_addr = 0x0;

    /* Power supply VDD   range: 1.71(Typ 1.8) to 3.6(Typ 3.3)*/
    uint16_t vdd_millivolt[] = {1800, 1900, 2100, 2800, 3000, 3300}; // Test VDD values
    /* Power supply VDDIO range: 1.2(Typ 1.8) to 3.6(Typ 3.3)*/
    uint16_t vddio_millivolt[] = {1800, 1900, 2100, 2800, 3000, 3300}; // Test VDDIO values

    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till USB is connnected

    /* Evaluate various VDD ranges while keeping the VDDIO fixed at 3.3V */
    for (uint8_t i = 0; i < sizeof(vdd_millivolt)/sizeof(vdd_millivolt[0]); i++)
    {
        printf("Configuring VDD: %.1fV\n", vdd_millivolt[i]/1000.00);
        coines_set_shuttleboard_vdd_vddio_config(vdd_millivolt[i], 3300);
        coines_delay_msec(1000);

        /* I2C config */
	    (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

	    /* I2C read */
	    (void)coines_read_i2c(COINES_I2C_BUS_0, BMI2_I2C_PRIM_ADDR, reg_addr, &chip_id, no_of_bytes);
	
	    printf("I2C read: Sensor chip ID - 0x%x\n", chip_id);
    }

    /* Evaluate various VDDIO ranges while keeping the VDD fixed at 3.3V*/
    for (uint8_t i = 0; i < sizeof(vddio_millivolt)/sizeof(vddio_millivolt[0]); i++)
    {
        printf("Configuring VDDIO: %.1fV\n", vddio_millivolt[i]/1000.00);
        coines_set_shuttleboard_vdd_vddio_config(3300, vddio_millivolt[i]);
        coines_delay_msec(1000);

        /* I2C config */
	    coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);

	    /* I2C read */
	    (void)coines_read_i2c(COINES_I2C_BUS_0, BMI2_I2C_PRIM_ADDR, reg_addr, &chip_id, no_of_bytes);
	
	    printf("I2C read: Sensor chip ID - 0x%x\n", chip_id);
    }
    
	(void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
	coines_delay_msec(100);
	
	/* Coines interface reset */
	coines_soft_reset();
	coines_delay_msec(100);

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    return (0);
}


