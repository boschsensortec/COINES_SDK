/**
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
 *
 * @file    decoder_support.c
 * @brief This layer provides functions for processing the commands from the host.
 *
 */
/*!
 * @ingroup APPLICATION
 *
 * @defgroup DECODER decoder
 * @{
 *
 *
 *
 **/

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "coines.h"
#include "app30_eeprom.h"

/**********************************************************************************/
/* own header files */
/**********************************************************************************/
#include "decoder.h"
#include "decoder_support.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
static uint16_t last_vdd;
static uint16_t last_vddio;

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/*!
*
* @brief       : API to validate the command parameters
*/
decoder_rsp_t parameter_prevalidation(const uint8_t *buffer)
{
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t mode = buffer[4];
    uint8_t bus_select = buffer[5];

    /* pre-validation of parameter values */
    if ((mode < 1) || (mode > 3))
    {
        rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
    }

    if (bus_select > 32)
    {
        rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
    }

    return rsp;

}

/*!
 *
 * @brief       : API to calculate the length
 */
uint8_t decoder_calc_len(const uint8_t *cmd)
{
    uint8_t length;
    {
        /* Length of the command is updated */
        length = cmd[DECODER_BYTEPOS_SIZE];

        /* check the length is greater than end indicators size */
        if (length > DECODER_END_INDICATORS_SIZE)
        {
            /* search for the CRLF cmd terminator in the received buffer */
            if ((cmd[length - 2] == DECODER_CR_MACRO) && (cmd[length - 1] == DECODER_LF_MACRO))
            {
                /* terminator found, command size is now determined */
                /*Actual length received in command is correct */
            }
            else
            {
                length = 0;
            }
        }
        else
        {
            length = 0;
        }

        return length;
    }
}

/*!
 *
 * @brief       : API will sets/gets sensor vdd voltage
 */
decoder_rsp_t decoder_0x01_vdd(const uint8_t *buffer, decoder_dir_t dir)
{
    uint16_t vdd_voltage = 0xFFFF;  /*TODO: temporary workaround */
    uint8_t vdd_state = 0;
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t terminating_char_pos;

    if (dir == DECODER_DIR_SET)
    {
        vdd_state = buffer[4];
        vdd_voltage = (uint16_t) (((buffer[5]) << 8) | (buffer[6]));

        /*TODO workaround after the voltage is updated 3300 mv support should be removed. */
        if ((vdd_voltage <= VDD_VDDIO_3V3) && (vdd_state <= 1))
        {
            if (vdd_state == 1)
            {
                /*Workaround to fix DD inputs */
                if (vdd_voltage == 0)
                {
                    vdd_voltage = VDD_VDDIO_2V8;
                }

                /* pass the voltage value*/
                coines_set_shuttleboard_vdd_vddio_config(vdd_voltage, last_vddio);
                last_vdd = vdd_voltage;

            }
            else
            {
                coines_set_shuttleboard_vdd_vddio_config(0, last_vddio);
                last_vdd = 0;
            }
        }
        else
        {
            rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
        }
    }

    /* Forming the Response */
    xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */
    xmit_buffer[1] = 11; /* Number  of bytes in the packet */
    xmit_buffer[2] = 1; /* Packet Number */
    xmit_buffer[3] = (uint8_t) rsp; /* Success message*/
    xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
    xmit_buffer[5] = (uint8_t) DECODER_ID_VDD; /* Feature of the Command */
    xmit_buffer[6] = vdd_state; /* vdd state */
    xmit_buffer[7] = (uint8_t) (vdd_voltage >> 8); /* vdd MSB value */
    xmit_buffer[8] = (uint8_t) (vdd_voltage); /* vdd LSB value */

    /* adds timestamp to the xmit_buffer if required */
    decoder_update_timestamp();

    terminating_char_pos = xmit_buffer[1] - 2;
    xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
    xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

    decoder_write_resp(xmit_buffer, xmit_buffer[1]);

    return rsp;

}

/*!
 *
 * @brief       : API will sets/gets sensor vddio voltage
 */
decoder_rsp_t decoder_0x02_vddio(const uint8_t *buffer, decoder_dir_t dir)
{
    uint16_t vddio_voltage = 0xFFFF;    /*TODO: temporary workaround */
    uint8_t vddio_state = 0;
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t terminating_char_pos;

    if (dir == DECODER_DIR_SET)
    {
        vddio_state = buffer[4];
        vddio_voltage = (uint16_t) (((buffer[5]) << 8) | (buffer[6]));

        /*TODO workaround after the voltage is updated 3300 mv support should be removed. */
        if ((vddio_voltage <= VDD_VDDIO_3V3) && (vddio_state <= 1))
        {
            /*TODO: */

            if (vddio_state == 1)
            {
                /*Workaround to fix DD inputs */
                if (vddio_voltage == 0)
                {
                    vddio_voltage = VDD_VDDIO_1V8;
                }

                coines_set_shuttleboard_vdd_vddio_config(last_vdd, vddio_voltage);
                last_vddio = vddio_voltage;

            }
            else
            {
                coines_set_shuttleboard_vdd_vddio_config(last_vdd, 0);
                last_vddio = 0;
            }
        }
        else
        {
            rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
        }
    }

    /* Forming the Response */
    xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */
    xmit_buffer[1] = 11; /* Number  of bytes in the packet */
    xmit_buffer[2] = 1; /* Packet Number */
    xmit_buffer[3] = (uint8_t) rsp; /* Success message*/
    xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
    xmit_buffer[5] = (uint8_t) DECODER_ID_VDDIO; /* Feature of the Command */

    xmit_buffer[6] = vddio_state; /* vdd state */
    xmit_buffer[7] = (uint8_t) (vddio_voltage >> 8); /* vdd MSB value */
    xmit_buffer[8] = (uint8_t) (vddio_voltage); /* vdd LSB value */

    /* adds timestamp to the xmit_buffer if required */
    decoder_update_timestamp();

    terminating_char_pos = xmit_buffer[1] - 2;
    xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
    xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

    decoder_write_resp(xmit_buffer, xmit_buffer[1]);

    return rsp;
}

/*!
 *
 * @brief       : API will sets/gets i2c speed
 */
decoder_rsp_t decoder_0x09_i2cspeed(const uint8_t *buffer, decoder_dir_t dir)
{
    uint8_t bus_index = 0, bus_speed = 0;
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t terminating_char_pos;

    if (dir == DECODER_DIR_SET)
    {
        bus_index = buffer[4];
        bus_speed = buffer[5];

        /* Deconfig the already configured i2c with default settings and configure with new settings */
        (void)coines_deconfig_i2c_bus((enum coines_i2c_bus)bus_index);

        if (bus_speed <= (uint8_t) COINES_I2C_FAST_MODE)
        {
            if (bus_index == 0)
            {
                if (coines_config_i2c_bus((enum coines_i2c_bus)bus_index,
                                          (enum coines_i2c_mode) bus_speed) != COINES_SUCCESS)
                {
                    rsp = DECODER_RSP_FAIL;
                }
            }
            else
            {
                rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
            }
        }
        else
        {
            rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
        }
    }
    else
    {
        rsp = DECODER_RSP_UNKNOWN_INSTR;
    }

    /* Forming the Response */
    xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */
    xmit_buffer[1] = 10; /* Number  of bytes in the packet */
    xmit_buffer[2] = 1; /* Packet Number */
    xmit_buffer[3] = (uint8_t) rsp; /* Success message*/
    xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
    xmit_buffer[5] = (uint8_t) DECODER_ID_I2CSPEED;
    xmit_buffer[6] = bus_index;
    xmit_buffer[7] = bus_speed;

    /* adds timestamp to the xmit_buffer if required */
    decoder_update_timestamp();

    terminating_char_pos = xmit_buffer[1] - 2;
    xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
    xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

    decoder_write_resp(xmit_buffer, xmit_buffer[1]);

    return rsp;

}

/*!
 *
 * @brief       : API to set/get the interfaces to communicate with sensors
 */
decoder_rsp_t decoder_0x11_interface(const uint8_t *buffer, decoder_dir_t dir)
{
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t terminating_char_pos;
    uint8_t sdo_level = 0;
    bool retval = true;

    /*
     * TODO: workaround for PCLint
     * (void)buffer;
     */

    /* Need to decide if the interface has to be set, whether we can keep the SPI interface enabled always*/

    /*TODO: Is it needed? */
    uint8_t interface = 0;

    if (dir == DECODER_DIR_SET)
    {
        interface = buffer[4]; /* Get the interface number*/

        if (interface == 0) /* SPI*/
        {
            if (coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_10_MHZ, COINES_SPI_MODE0) != COINES_SUCCESS)
            {
                rsp = DECODER_RSP_FAIL;
            }

            coines_deconfig_i2c_bus(COINES_I2C_BUS_0);

        }
        else if (interface == 1)   /* I2C*/
        {
            sdo_level = buffer[5];
            coines_deconfig_spi_bus(COINES_SPI_BUS_0);

#if (defined(MCU_APP30)||defined(MCU_APP31))
            coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_3,
                                  COINES_PIN_DIRECTION_OUT,
                                  (enum coines_pin_value)sdo_level);
#else
            (void)sdo_level;
#endif

            if (coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE))
            {
                rsp = DECODER_RSP_FAIL;
            }
        }
        else   /* SPI & I2C*/
        {
            /* TODO: Add COINES_SDK support for this feature. Multiple SPI & I2C */
        }
    }

    if (!retval) /*lint !e774 */
    {
        rsp = DECODER_RSP_FAIL;
    }

    /* Forming the Response */
    xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */
    xmit_buffer[1] = 8; /* Number  of bytes in the packet */
    xmit_buffer[2] = 1; /* Packet Number */
    xmit_buffer[3] = (uint8_t) rsp; /* Success message*/
    xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
    xmit_buffer[5] = (uint8_t) DECODER_ID_INTERFACE; /* Feature of the Command */

    /* adds timestamp to the xmit_buffer if required */
    decoder_update_timestamp();

    terminating_char_pos = xmit_buffer[1] - 2;
    xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
    xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

    decoder_write_resp(xmit_buffer, xmit_buffer[1]);

    return rsp;

}

/*!
 *
 * @brief       : API to set shuttle vdd and vddio voltages
 */
decoder_rsp_t decoder_0x14_shuttle_pwr_cfg(const uint8_t *buffer, decoder_dir_t dir)
{
    uint8_t vdd_state = 0, vddio_state = 0;
    uint16_t vdd_voltage = 0, vddio_voltage = 0;
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t terminating_char_pos;

    /* TODO vdd set and get actual implementation */
    if (dir == DECODER_DIR_SET)
    {
        vdd_voltage = (uint16_t) ((buffer[4] << 8) | buffer[5]);
        vdd_state = buffer[6];
        vddio_voltage = (uint16_t) ((buffer[7] << 8) | buffer[8]);
        vddio_state = buffer[9];

        if ((vdd_voltage <= VDD_VDDIO_3V3) && (vdd_state <= 1) && (vddio_voltage <= VDD_VDDIO_3V3) &&
            (vddio_state <= 1))
        {
            /*TODO: */
            if (vdd_state == 1)
            {
                /*Workaround to fix DD inputs */
                if (vdd_voltage == 0)
                {
                    vdd_voltage = VDD_VDDIO_2V8;
                }

                coines_set_shuttleboard_vdd_vddio_config(vdd_voltage, last_vddio);
                last_vdd = vdd_voltage;
            }
            else
            {
                coines_set_shuttleboard_vdd_vddio_config(0, last_vddio);
                last_vdd = 0;
            }

            if (vddio_state == 1)
            {
                /*Workaround to fix DD inputs */
                if (vddio_voltage == 0)
                {
                    vddio_voltage = VDD_VDDIO_1V8;
                }

                coines_set_shuttleboard_vdd_vddio_config(last_vdd, vddio_voltage);
                last_vddio = vddio_voltage;
            }
            else
            {
                coines_set_shuttleboard_vdd_vddio_config(last_vdd, 0);
                last_vddio = 0;
            }
        }
        else
        {
            rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
        }
    }
    else
    {
        /* workaround implementation */
        /*TODO: Get command not listed in commands list, do we need to keep this ? */
        if (dir == DECODER_DIR_GET)
        {
            vdd_voltage = 0;
            vdd_state = 0;
            vddio_voltage = 0;
            vddio_state = 0;
        }
    }

    /* Forming the Response */
    xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */
    xmit_buffer[1] = 14; /* Number  of bytes in the packet */
    xmit_buffer[2] = 1; /* Packet Number */
    xmit_buffer[3] = (uint8_t) (rsp); /* Success message*/
    xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
    xmit_buffer[5] = (uint8_t) DECODER_ID_SHUTTLE_PWR_CFG; /* Feature of the Command */

    xmit_buffer[6] = (uint8_t) (vdd_voltage >> 8); /* vdd voltage MSB */
    xmit_buffer[7] = (uint8_t) (vdd_voltage); /* vdd voltage LSB */
    xmit_buffer[8] = vdd_state; /* vdd state */
    xmit_buffer[9] = (uint8_t) (vddio_voltage >> 8); /* vddio voltage MSB */
    xmit_buffer[10] = (uint8_t) (vddio_voltage); /* vddio voltage LSB */
    xmit_buffer[11] = vddio_state; /* vddio state */

    /* adds timestamp to the xmit_buffer if required */
    decoder_update_timestamp();

    terminating_char_pos = xmit_buffer[1] - 2;
    xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
    xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

    decoder_write_resp(xmit_buffer, xmit_buffer[1]);

    return rsp;

}

/*!
 *
 * @brief       : API to set/get IO pins
 *                APP2.0 - set/get multiple IOs status at once
 *                APP3.0 - set/get only one pin at a time !
 *                         If 15th bit is set, then pin is of APP3.0's !
 */
decoder_rsp_t decoder_0x15_multiioconfig_support(const uint8_t* buffer,
                                                 uint16_t *gpio_sel_mask,
                                                 uint16_t *gpio_dir_mask,
                                                 uint16_t *gpio_state_mask)
{
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint16_t pin_mask;
    enum coines_multi_io_pin i;
    enum coines_pin_direction pin_dir;
    enum coines_pin_value pin_state;
    int16_t retval = false;
    enum coines_pin_direction gpio_dir;
    enum coines_pin_value gpio_level;

    *gpio_dir_mask = (buffer[6] << 8) | buffer[7];
    *gpio_state_mask = (buffer[8] << 8) | buffer[9];

    if ((*gpio_sel_mask & APP30_SHUTTLE_PIN_ID) == 0)
    {
        for (i = COINES_SHUTTLE_PIN_9; i <= DECODER_MAX_CS_PIN; i++)
        {
            pin_mask = (1 << (i & 0x0F));
            if (*gpio_sel_mask & pin_mask)
            {
                /* GPIO i is requested, determine direction and state */
                pin_dir = (*gpio_dir_mask & pin_mask) ?COINES_PIN_DIRECTION_OUT : COINES_PIN_DIRECTION_IN;
                pin_state = (*gpio_state_mask & pin_mask) ?COINES_PIN_VALUE_HIGH : COINES_PIN_VALUE_LOW;

                /* configure GPIO i */
                retval = coines_set_pin_config(i, pin_dir, pin_state);

                if (retval == COINES_SUCCESS)
                {
                    /* read back the pin state to insure pin was correctly configured */
                    retval = coines_get_pin_config(i, &gpio_dir, &gpio_level);
                    if ((retval != COINES_SUCCESS) || (gpio_dir != pin_dir) || (gpio_level != pin_state))
                    {
                        rsp = DECODER_RSP_FAIL;
                    }
                }
                else
                {
                    rsp = DECODER_RSP_FAIL;
                }
            }
        }
    }
    else /* APP3.0 shuttle pin */
    {
        *gpio_sel_mask &= ~APP30_SHUTTLE_PIN_ID;
        pin_dir = *gpio_dir_mask ? COINES_PIN_DIRECTION_OUT : COINES_PIN_DIRECTION_IN;
        pin_state = *gpio_state_mask ? COINES_PIN_VALUE_HIGH : COINES_PIN_VALUE_LOW;
        if (coines_set_pin_config((enum coines_multi_io_pin)*gpio_sel_mask, pin_dir, pin_state) != COINES_SUCCESS)
        {
            rsp = DECODER_RSP_FAIL;
        }
    }

    return rsp;
}

/*!
 *
 * @brief       : API to set/get IO pins
 *                APP2.0 - set/get multiple IOs status at once
 *                APP3.0 - set/get only one pin at a time !
 *                         If 15th bit is set, then pin is of APP3.0's !
 */
decoder_rsp_t decoder_0x15_multiioconfig(const uint8_t* buffer, decoder_dir_t dir)
{
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint16_t gpio_sel_mask, gpio_dir_mask, gpio_state_mask, pin_mask;
    enum coines_multi_io_pin i;
    enum coines_pin_direction pin_dir;
    enum coines_pin_value pin_state;

    uint8_t terminating_char_pos;
    int16_t retval = COINES_SUCCESS;

    gpio_sel_mask = (buffer[4] << 8) | buffer[5];

    if (dir == DECODER_DIR_SET)
    {
        rsp = decoder_0x15_multiioconfig_support(buffer, &gpio_sel_mask, &gpio_dir_mask, &gpio_state_mask);
    }
    else
    {
        gpio_dir_mask = 0;
        gpio_state_mask = 0;
        if ((gpio_sel_mask & APP30_SHUTTLE_PIN_ID) == 0)
        {

            for (i = COINES_SHUTTLE_PIN_9; i < DECODER_MAX_CS_PIN; i++)
            {
                pin_mask = (1 << (i & 0x0F));
                if (gpio_sel_mask & pin_mask)
                {
                    /* GPIO i is requested, determine direction and state */

                    /* read the pin state */
                    retval = coines_get_pin_config(i, &pin_dir, &pin_state);
                    if (retval == COINES_SUCCESS)
                    {
                        if (pin_dir == COINES_PIN_DIRECTION_OUT)
                        {
                            gpio_dir_mask |= pin_mask;
                        }

                        if (pin_state == COINES_PIN_VALUE_HIGH)
                        {
                            gpio_state_mask |= pin_mask;
                        }
                    }
                    else
                    {
                        rsp = DECODER_RSP_FAIL;
                    }
                }
            }
        }
        else   /* APP3.0 shuttle pin */
        {
            gpio_sel_mask &= ~APP30_SHUTTLE_PIN_ID;
            if (coines_get_pin_config((enum coines_multi_io_pin)gpio_sel_mask, &pin_dir, &pin_state) == COINES_SUCCESS)
            {
                gpio_dir_mask = (uint16_t) pin_dir;
                gpio_state_mask = (uint16_t) pin_state;
            }
            else
            {
                rsp = DECODER_RSP_FAIL;
            }
        }
    }

    /* Forming the Response */
    xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */

    xmit_buffer[2] = 1; /* Packet Number */
    xmit_buffer[3] = (uint8_t) rsp; /* Success message*/
    xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
    xmit_buffer[5] = (uint8_t) DECODER_ID_MULTIIOCONFIG; /* Feature of the Command */

    if (dir == DECODER_DIR_SET)
    {
        xmit_buffer[1] = 8; /* Number  of bytes in the packet */
    }
    else
    {
        xmit_buffer[1] = 14; /* Number  of bytes in the packet */
        xmit_buffer[6] = (uint8_t) (gpio_sel_mask >> 8);
        xmit_buffer[7] = (uint8_t) gpio_sel_mask;
        xmit_buffer[8] = (uint8_t) (gpio_dir_mask >> 8);
        xmit_buffer[9] = (uint8_t) gpio_dir_mask;
        xmit_buffer[10] = (uint8_t) (gpio_state_mask >> 8);
        xmit_buffer[11] = (uint8_t) gpio_state_mask;
    }

    /* adds timestamp to the xmit_buffer if required */
    decoder_update_timestamp();

    terminating_char_pos = xmit_buffer[1] - 2;
    xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
    xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

    decoder_write_resp(xmit_buffer, xmit_buffer[1]);

    return rsp;
}

/*!
 *
 * @brief       : API to set/get spi configurations
 */
decoder_rsp_t decoder_0x19_spiconfig(const uint8_t *buffer, decoder_dir_t dir)
{
    uint8_t bus_index = 0, bus_speed = 0, bus_mode = 0, bus_length = 0;
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t terminating_char_pos;

    /*check if it is a command set*/
    if (dir == DECODER_DIR_SET)
    {
        /* get the bus index, spi mode and spi speed*/
        bus_index = buffer[4];
        bus_mode = buffer[5];
        bus_length = buffer[6];
        bus_speed = buffer[7];

        /* Deconfig the already configured spi with default settings and configure with new settings */
        (void)coines_deconfig_spi_bus(COINES_SPI_BUS_0);

        /* check if the bus index is 1*/
        if (bus_index == 1)
        {
            /* check if it is valid SPI bus mode*/
            if (bus_mode < 4)
            {
                if (coines_config_spi_bus(COINES_SPI_BUS_0, (enum coines_spi_speed)bus_speed,
                                          (enum coines_spi_mode)bus_mode) != COINES_SUCCESS)
                {
                    rsp = DECODER_RSP_FAIL;
                }
            }
            else
            {
                rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
            }
        }
        else
        {
            rsp = DECODER_RSP_PARAM_OUT_OF_RANGE;
        }
    }
    else
    {
        rsp = DECODER_RSP_UNKNOWN_INSTR;
    }

    /* Forming the Response */
    xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */
    xmit_buffer[1] = 12; /* Number  of bytes in the packet */
    xmit_buffer[2] = 1; /* Packet Number */
    xmit_buffer[3] = (uint8_t) rsp; /* Success message*/
    xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
    xmit_buffer[5] = (uint8_t) DECODER_ID_SPICONFIG;
    xmit_buffer[6] = bus_index;
    xmit_buffer[7] = bus_mode;
    xmit_buffer[8] = bus_length;
    xmit_buffer[9] = bus_speed;

    /* adds timestamp to the xmit_buffer if required */
    decoder_update_timestamp();

    terminating_char_pos = xmit_buffer[1] - 2;
    xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
    xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

    /* Function is void, since possibility of overflowing for non-streaming command is less*/
    decoder_write_resp(xmit_buffer, xmit_buffer[1]);

    return rsp;

}

/*!
 *
 * @brief       : API used to get the board information
 */
decoder_rsp_t decoder_0x1f_boardinfo(const uint8_t *buffer, decoder_dir_t dir)
{
    struct coines_board_info board_info;

    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;
    uint8_t terminating_char_pos;

    (void) buffer;

    if (dir == DECODER_DIR_GET)
    {
        coines_get_board_info(&board_info);

        /* Forming the Response */
        xmit_buffer[0] = DECODER_HEADER_VALUE; /* Start of Command */
        xmit_buffer[1] = 15; /* Number  of bytes in the packet */
        xmit_buffer[2] = 1; /* Packet Number */
        xmit_buffer[3] = (uint8_t) rsp; /* Success message*/
        xmit_buffer[4] = (uint8_t) dir | DECODER_RSP_ID_MASK; /* Direction of the Command */
        xmit_buffer[5] = (uint8_t) DECODER_ID_BOARDINFO; /* Feature of the Command */
        xmit_buffer[6] = (uint8_t) (board_info.shuttle_id >> 8); /* SID MSB */
        xmit_buffer[7] = (uint8_t) (board_info.shuttle_id); /* SID LSB */
        xmit_buffer[8] = (uint8_t) (board_info.hardware_id >> 8); /* HWID MSB */
        xmit_buffer[9] = (uint8_t) (board_info.hardware_id); /* HWID LSB */
        xmit_buffer[10] = (uint8_t) (SOFTWARE_VERSION_MAJOR & 0xFF); /* SWID MSB */
        xmit_buffer[11] =
            (uint8_t) (((SOFTWARE_VERSION_MINOR & 0x0F) << 4) | (SOFTWARE_VERSION_PATCH & 0x0F)); /* SWID LSB */
        xmit_buffer[12] = board_info.board; /* Board Info */

        /* adds timestamp to the xmit_buffer if required */
        decoder_update_timestamp();

        terminating_char_pos = xmit_buffer[1] - 2;
        xmit_buffer[terminating_char_pos++] = DECODER_CR_MACRO; /* \r */
        xmit_buffer[terminating_char_pos] = DECODER_LF_MACRO;/* \n */

        decoder_write_resp(xmit_buffer, xmit_buffer[1]);
    }
    else
    {
        rsp = DECODER_RSP_UNKNOWN_INSTR;
    }

    return rsp;

}

/* Regarding info. on Application Switch,
 * see https://bcds02.de.bosch.com/confluence/display/DEV/USB+DFU+Bootloader
 * (Invoking Bootloader from software)
 */
/*!
 *
 * @brief       : API to set switch from application to BTL
 */
decoder_rsp_t decoder_0x30_app_switch(const uint8_t *buffer, decoder_dir_t dir)
{
    uint32_t appstart_addr = 0;

    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;

    if (dir == DECODER_DIR_SET)
    {
        appstart_addr = (uint32_t)(buffer[4] << 24);
        appstart_addr |= buffer[5] << 16;
        appstart_addr |= buffer[6] << 8;
        appstart_addr |= buffer[7];

        if (appstart_addr == 0)
        {
            appstart_addr = 0xF0000;
        }

        if (((appstart_addr >= 0x28000 && appstart_addr < 0x100000) || /*Check for valid flash address*/
             (appstart_addr >= 0x20000000 && appstart_addr < 0x20040000)) &&    /*Check for valid RAM address*/
            appstart_addr % 2 == 0     /*Check if address if even*/
            )
        {
#ifndef UNIT_TEST
            memcpy(DECODER_BTL_MAGIC_INFO_ADDR, DECODER_BTL_MAGIC_DATA, 4); /* Write Magic info. "COIN" */
            DECODER_BTL_APP_START_ADDR = appstart_addr;
            if (DECODER_APP_SP_VALUE > 0x20000000 && DECODER_APP_SP_VALUE < 0x20040000) /*Check if the application has
                                                                                         * vaild Stack pointer value*/
            {
                coines_delay_msec(10);
                coines_soft_reset();
            }
            else
            {
                rsp = decoder_error_fuction(DECODER_RSP_INVALID_APPLICATION,
                                            (int8_t) DECODER_ID_APP_SWITCH,
                                            (int8_t) dir);                                 /*Invaild application */
            }

#endif
        }
        else
        {
            rsp = decoder_error_fuction(DECODER_RSP_INVALID_ADDRESS, (int8_t) DECODER_ID_APP_SWITCH, (int8_t) dir); /*Invaild
                                                                                                                     * address */
        }
    }
    else
    {
        rsp = decoder_error_fuction(DECODER_RSP_UNKNOWN_INSTR, (int8_t) DECODER_ID_APP_SWITCH, (int8_t) dir);
    }

    return rsp;
}

/*!
 *
 * @brief       : API to invoke BTL
 */
decoder_rsp_t decoder_0x31_invoke_btl(const uint8_t *buffer, decoder_dir_t dir)
{
    decoder_rsp_t rsp = DECODER_RSP_SUCCESS;

    (void) buffer;

    if (dir != DECODER_DIR_SET)
    {
        rsp = decoder_error_fuction(DECODER_RSP_UNKNOWN_INSTR, (int8_t) DECODER_ID_INVOKE_BTL, (int8_t) dir);
    }
    else
    {
        memcpy(DECODER_BTL_MAGIC_INFO_ADDR, DECODER_BTL_MAGIC_DATA, 4); /* Write Magic info. "COIN" */
        DECODER_BTL_APP_START_ADDR = 0;
        coines_delay_msec(10);
        coines_soft_reset();
    }

    return rsp;
}
