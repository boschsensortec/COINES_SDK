/*!
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
 * @file board_interface.c
 *
 * @brief This module provides APIs for handling data transfers with different serial interfaces
 *
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "board_interface.h"

/*********************************************************************/
/* header files */
/*********************************************************************/
#include "coines.h"
#include "api.h"
#include "protocol.h"
#include "error_handling.h"
#include "coines_common.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/

/*********************************************************************/
/* constant definitions */
/*********************************************************************/

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static variables */
/*********************************************************************/

/*********************************************************************/
/* extern variables */
/*********************************************************************/
extern enum coines_comm_intf interface_type;
extern uint8_t *resp_buffer;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static bool is_sys_little_endian = true;

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This API is used to configure the spi bus
 *
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
{
    uint8_t payload[3] = { bus, spi_speed, spi_mode };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SPI_CONFIG, payload, 3);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SPI_CONFIG, resp_buffer, &resp_length);

        is_sys_little_endian = is_system_little_endian();
    }

    return get_coines_error_mapping(ret);
}

/*!
 *  @brief This API is used to de-configure the spi bus
 *
 */
int16_t coines_deconfig_spi_bus(enum coines_spi_bus bus)
{
    uint8_t payload = { bus };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SPI_DECONFIG, &payload, 1);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SPI_DECONFIG, resp_buffer, &resp_length);
    }

    return get_coines_error_mapping(ret);
}

/*!
 *  @brief This API is used to configure the spi bus with 8 bit or 16 bit length
 *
 */
int16_t coines_config_word_spi_bus(enum coines_spi_bus bus,
                                   enum coines_spi_speed spi_speed,
                                   enum coines_spi_mode spi_mode,
                                   enum coines_spi_transfer_bits spi_transfer_bits)
{
   /*
   Nordic's SPI initialization function is the same for both 8-bit and 16-bit transfers. 
   */
  (void)spi_transfer_bits;
   return coines_config_spi_bus(bus, spi_speed, spi_mode); 
}

/*!
 *  @brief This API is used to configure the i2c bus
 *
 */
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode)
{
    uint8_t payload[2] = { bus, i2c_mode };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_I2C_CONFIG, payload, 2);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_I2C_CONFIG, resp_buffer, &resp_length);

        is_sys_little_endian = is_system_little_endian();
    }

    return get_coines_error_mapping(ret);
}

/*!
 *  @brief This API is used to de-configure the i2c bus
 *
 */
int16_t coines_deconfig_i2c_bus(enum coines_i2c_bus bus)
{
    uint8_t payload = bus;
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_I2C_DECONFIG, &payload, 1);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_I2C_DECONFIG, resp_buffer, &resp_length);
    }

    return get_coines_error_mapping(ret);
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 *
 */
int8_t coines_write_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    uint8_t payload[5] = { bus, dev_addr, reg_addr, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;

    memcpy(&payload[3], &count, 2);

    ret = protocol_encode_multi_packet(interface_type, COINES_CMD_ID_I2C_WRITE_REG, payload, 5, reg_data, count);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_I2C_WRITE_REG, resp_buffer, &resp_length);
    }

    return (int8_t)(get_coines_error_mapping(ret));
}

/*!
 *  @brief This API is used to write 16-bit register data on the I2C device.
 *
 */
int8_t coines_write_16bit_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    uint8_t reg_addr_lsb = reg_addr & 0xFF;
    uint8_t reg_addr_msb = (reg_addr >> 8) & 0xFF;
    uint8_t payload_len = 6, data_len_index = 4;
    uint8_t payload[6] = { bus, dev_addr, reg_addr_lsb, reg_addr_msb, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;
    uint16_t swapped_reg_data[count];
    uint32_t data_length;
    uint8_t *write_reg_data;

    if(i2c_transfer_bits == COINES_I2C_TRANSFER_16BIT)
    {
        data_length = count * sizeof(uint16_t);

        if(is_sys_little_endian){
            swap_endianness(swapped_reg_data, (uint16_t*)reg_data, count);
        }else{
            memcpy(swapped_reg_data, (uint16_t*)reg_data, count);
        }
        write_reg_data = (uint8_t*)swapped_reg_data;
    }
    else 
    {
        data_length = count;
        write_reg_data = (uint8_t*)reg_data;
    }

    memcpy(&payload[data_len_index], &data_length, 2);

    ret = protocol_encode_multi_packet(interface_type, COINES_CMD_ID_I2C_WRITE_REG_16, payload, payload_len, write_reg_data, data_length);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_I2C_WRITE_REG_16, resp_buffer, &resp_length);
    }

    return (int8_t)(get_coines_error_mapping(ret));
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 *
 */
int8_t coines_read_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    uint8_t payload[5] = { bus, dev_addr, reg_addr, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;

    memcpy(&payload[3], &count, 2);

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_I2C_READ_REG, payload, 5);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_I2C_READ_REG, resp_buffer, &resp_length);
        if (resp_length != count)
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }

        memcpy(reg_data, &resp_buffer[PAYLOAD_POS], resp_length);
    }

    return (int8_t)(get_coines_error_mapping(ret));
}

/*!
 *  @brief This API is used to read 16-bit register data from the I2C device.
 *
 */
int8_t coines_read_16bit_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    uint8_t reg_addr_lsb = reg_addr & 0xFF;
    uint8_t reg_addr_msb = (reg_addr >> 8) & 0xFF;
    uint8_t payload_len = 6, data_len_index = 4;
    uint8_t payload[6] = { bus, dev_addr, reg_addr_lsb, reg_addr_msb, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;
    uint32_t data_length;

    if(i2c_transfer_bits == COINES_I2C_TRANSFER_16BIT)
    {
        data_length = count * sizeof(uint16_t);
    }
    else 
    {
        data_length = count;
    }

    memcpy(&payload[data_len_index], &data_length, 2);

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_I2C_READ_REG_16, payload, payload_len);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_I2C_READ_REG_16, resp_buffer, &resp_length);
        if (resp_length != data_length)
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }

        memcpy((uint8_t*)reg_data, &resp_buffer[PAYLOAD_POS], resp_length);

        if(i2c_transfer_bits == COINES_I2C_TRANSFER_16BIT && is_sys_little_endian){
            /* Convert the payload from little endian to big endian */
            swap_endianness((uint16_t*)reg_data, (uint16_t*)reg_data, count);
        }
    }

    return (int8_t)(get_coines_error_mapping(ret));
}
/*!
 *  @brief This API is used to write 16-bit register data on the SPI device.
 *
 */
int8_t coines_write_16bit_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    uint8_t reg_addr_lsb = reg_addr & 0xFF;
    uint8_t reg_addr_msb = (reg_addr >> 8) & 0xFF;
    uint8_t payload_len = 6, data_len_index = 4;
    uint8_t payload[6] = { bus, cs_pin, reg_addr_lsb, reg_addr_msb, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;
    uint16_t swapped_reg_data[count];
    uint32_t data_length;
    uint8_t *write_reg_data;

    if(spi_transfer_bits == COINES_SPI_TRANSFER_16BIT)
    {
        data_length = count * sizeof(uint16_t);

        if(is_sys_little_endian){
            swap_endianness(swapped_reg_data, (uint16_t*)reg_data, count);
        }else{
            memcpy(swapped_reg_data, (uint16_t*)reg_data, count);
        }
        write_reg_data = (uint8_t*)swapped_reg_data;
    }
    else 
    {
        data_length = count;
        write_reg_data = (uint8_t*)reg_data;
    }

    memcpy(&payload[data_len_index], &data_length, 2);

    ret = protocol_encode_multi_packet(interface_type, COINES_CMD_ID_SPI_WRITE_REG_16, payload, payload_len, write_reg_data, data_length);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SPI_WRITE_REG_16, resp_buffer, &resp_length);
    }

    return (int8_t)(get_coines_error_mapping(ret));
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 *
 */
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    uint8_t payload[5] = { bus, cs_pin, reg_addr, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;

    memcpy(&payload[3], &count, 2);

    ret = protocol_encode_multi_packet(interface_type, COINES_CMD_ID_SPI_WRITE_REG, payload, 5, reg_data, count);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SPI_WRITE_REG, resp_buffer, &resp_length);
    }

    return (int8_t)(get_coines_error_mapping(ret));
}

/*!
 *  @brief This API is used to read 16-bit register data from the SPI device.
 *
 */
int8_t coines_read_16bit_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    uint8_t reg_addr_lsb = reg_addr & 0xFF;
    uint8_t reg_addr_msb = (reg_addr >> 8) & 0xFF;
    uint8_t payload_len = 6, data_len_index = 4;
    uint8_t payload[6] = { bus, cs_pin, reg_addr_lsb, reg_addr_msb, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;
    uint32_t data_length;

    if(spi_transfer_bits == COINES_SPI_TRANSFER_16BIT)
    {
        data_length = count * sizeof(uint16_t);
    }
    else 
    {
        data_length = count;
    }

    memcpy(&payload[data_len_index], &data_length, 2);

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SPI_READ_REG_16, payload, payload_len);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SPI_READ_REG_16, resp_buffer, &resp_length);
        if (resp_length != data_length)
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }

        memcpy((uint8_t*)reg_data, &resp_buffer[PAYLOAD_POS], resp_length);

        if(spi_transfer_bits == COINES_SPI_TRANSFER_16BIT && is_sys_little_endian){
            /* Convert the payload from little endian to big endian */
            swap_endianness((uint16_t*)reg_data, (uint16_t*)reg_data, count);
        }
    }

    return (int8_t)(get_coines_error_mapping(ret));
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 *
 */
int8_t coines_read_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    uint8_t payload[5] = { bus, cs_pin, reg_addr, 0, 0 };
    int16_t ret;
    uint16_t resp_length = 0;

    memcpy(&payload[3], &count, 2);

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SPI_READ_REG, payload, 5);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SPI_READ_REG, resp_buffer, &resp_length);
        if (resp_length != count)
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }

        memcpy(reg_data, &resp_buffer[PAYLOAD_POS], resp_length);
    }

    return (int8_t)(get_coines_error_mapping(ret));
}
