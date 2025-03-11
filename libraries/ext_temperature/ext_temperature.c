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
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "ext_temperature.h"

/*! Structure to hold interface configurations */
static struct external_temp_intf_config temp_intf_conf;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES_SDK platform
 */
int8_t external_temp_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct external_temp_intf_config intf_info = *(struct external_temp_intf_config *)intf_ptr;

    return coines_read_i2c(intf_info.external_temp_bus, intf_info.external_temp_dev_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * I2C write function map to COINES_SDK platform
 */
int8_t external_temp_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct external_temp_intf_config intf_info = *(struct external_temp_intf_config *)intf_ptr;

    return coines_write_i2c(intf_info.external_temp_bus, intf_info.external_temp_dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * Delay function map to COINES_SDK platform
 */
void external_temp_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/*!
 * Function to initialize the COINES_SDK platform
 */
int8_t external_temp_coines_init(enum coines_comm_intf intf_type)
{
    int16_t rslt = EXTERNAL_TEMP_SUCCESS;

    rslt = coines_open_comm_intf(intf_type, NULL);
    
    if(EXTERNAL_TEMP_SUCCESS != rslt)
    {
        return EXTERNAL_TEMP_E_COMM_INIT_FAILED;
    }

    /* Power up the board */
    coines_set_shuttleboard_vdd_vddio_config(1800, 1800);

    coines_delay_msec(200);

    return rslt;
}

/*!
 * Function to initialize I2C interface for external temperature sensor
 */
int8_t external_temp_interface_init(struct external_temp_dev *temp_dev)
{
    int8_t rslt = EXTERNAL_TEMP_SUCCESS;

    if(temp_dev != NULL)
    {
        printf("I2C Interface \n");

        /* To initialize the user I2C function */
        temp_dev->intf = EXTERNAL_TEMP_I2C_INTF;
        temp_dev->temp_read = external_temp_i2c_read;
        temp_dev->temp_write = external_temp_i2c_write;

        coines_delay_msec(100);

        if(true == coines_is_i2c_enabled(COINES_I2C_BUS_1))
        {
            rslt = EXTERNAL_TEMP_SUCCESS;
        }
        else
        {
            /* Configures for I2C bus 1, speed 400KHz, pin mapping for reading temperature */
            rslt = coines_config_i2c_bus_internal(COINES_I2C_BUS_1, COINES_I2C_FAST_MODE, COINES_I2C_PIN_INTERNAL_TEMP);
        }
        
        temp_intf_conf.external_temp_bus = COINES_I2C_BUS_1;
        temp_intf_conf.external_temp_dev_addr = EXTERNAL_TEMP_I2C_ADDR;

        /* Assign device address to interface pointer */
        temp_dev->intf_ptr = ((void *)&temp_intf_conf);

        /* Configure delay in microseconds */
        temp_dev->temp_delay_us = external_temp_delay_us;
    }
    else
    {
        rslt = EXTERNAL_TEMP_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * Function to read external temperature sensor data
 */
int8_t external_temp_get_sensor_data(struct external_temp_dev *temp_dev, float *temp_data)
{
    int8_t rslt = EXTERNAL_TEMP_SUCCESS;

    uint8_t temp_buffer[2];
    uint16_t temp_raw_data;

    /* Null-pointer check */
    if ((temp_dev != NULL) && (temp_data != NULL))
    {
        temp_dev->intf_rslt = temp_dev->temp_read(EXTERNAL_TEMP_REG_ADDR, temp_buffer, EXTERNAL_TEMP_READ_LEN, temp_dev->intf_ptr);

        if (temp_dev->intf_rslt == EXTERNAL_TEMP_SUCCESS)
        {
            temp_raw_data = (uint16_t)temp_buffer[0];
            temp_raw_data = (uint16_t)((temp_raw_data << 4) | (temp_buffer[1] >> 4));  /* 12-bit data format */

            if (temp_raw_data & (1 << 11))
            {
                /* Negative temperature */
                temp_raw_data = ~temp_raw_data;
                temp_raw_data &= 0xFFF;
                temp_raw_data += 1;
                *temp_data = (float)temp_raw_data * 0.0625;
            }
            else
            {
                /* Positive temperature */
                *temp_data = (float)temp_raw_data * 0.0625;
            }
        }

        rslt = temp_dev->intf_rslt;
    }
    else
    {
        rslt = EXTERNAL_TEMP_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Deinitializes COINES_SDK platform
 *
 *  @return void.
 */
void external_temp_coines_deinit(enum coines_comm_intf intf)
{
    coines_set_shuttleboard_vdd_vddio_config(0, 0);

    coines_close_comm_intf(intf, NULL);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void external_temp_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != EXTERNAL_TEMP_SUCCESS)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case EXTERNAL_TEMP_E_COMM_IO_ERROR:
                printf("Error [%d] : Communication read/write failure.", rslt);
                printf(
                    "It occurs when the user tries to read or write value to the device.\r\n");
                break;

            case EXTERNAL_TEMP_E_COMM_INIT_FAILED:
                printf("Error [%d] : COMM Initialization error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case EXTERNAL_TEMP_E_DEVICE_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case EXTERNAL_TEMP_E_NOT_SUPPORTED:
                printf("Error [%d] : Invalid sensor configuration.", rslt);
                printf(" It occurs when there is a mismatch in the requested feature with the available one\r\n");
                break;

            case EXTERNAL_TEMP_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}