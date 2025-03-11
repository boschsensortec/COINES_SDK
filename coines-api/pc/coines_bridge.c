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
 *  @file   coines_bridge.c
 *  @brief  This module defines COINES_SDK APIs to be used by the user applications
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#ifdef PLATFORM_LINUX

/* To enable usleep in glibc */
#define _DEFAULT_SOURCE
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#else
#include <unistd.h>
#include <ctype.h>
#endif

/*********************************************************************/
/* other header files */
/**********************************************************************/
#include "coines.h"
#include "serial_com.h"
#include "ble_com.h"

/*********************************************************************/
/* own header files */
/**********************************************************************/
#include "coines_bridge.h"

/*********************************************************************/
/* local macro definitions */

/********************************************************************/
#define COM_OK  INT8_C(0)

/*********************************************************************/

/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static variables */
/*********************************************************************/
/*! Buffer to hold the response data */
static uint8_t *resp_buffer;

/*! Command packet buffer initialized to zero */
static uint8_t cmd_packet[COINES_BUFFER_SIZE] = { 0 };

/*! Size of the command packet */
static uint16_t cmd_packet_size;

/*! Buffer to hold the formatted date-time string */
static char datetime_str[64];

/*! Length of the formatted date-time string */
static uint16_t datetime_str_len;

/*! Variable to hold sensor count */
static uint8_t coines_sensor_id_count = 0;

/*! Variable to hold read status */
static int8_t com_read_status = COM_OK;

/*! Variable to hold write status */
static int8_t com_write_status = COM_OK;

/*! Variable to hold streaming sensor info */
static struct coines_stream_sensor_info coines_sensor_info;

/*! variable to hold the maximum no of streaming configuration buffer*/
static struct coines_streaming_settings coines_streaming_cfg_buf[COINES_MAX_SENSOR_ID];

/*! Variable to hold the communication interface type */
static enum coines_comm_intf comm_intf = COINES_COMM_INTF_USB;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static int16_t coines_connect_usb(struct coines_serial_com_config *scom_config);
static int16_t coines_disconnect_usb(void);
static int16_t coines_connect_ble(struct ble_peripheral_info *ble_config);
static int16_t coines_disconnect_ble(void);
static int16_t coines_send_packet(uint8_t command, uint8_t *payload, uint16_t length);
static int16_t read_data(uint8_t *resp_buffer, uint16_t packet_length);

/*! COINES_SDK config streaming mode */
static int16_t config_poll_stream_timing(void);
static int16_t dma_stream_config(void);
/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 * @brief This API is used to map BLE error codes to COINES error codes
 *
 */
static int16_t map_ble_error_codes(int16_t err_code)
{
    int16_t coines_ret = COINES_SUCCESS;

    switch (err_code)
    {
        case BLE_COM_OK:
            coines_ret = COINES_SUCCESS;
            break;
        case BLE_COM_E_ADAPTOR_NOT_FOUND:
            coines_ret = COINES_E_BLE_ADAPTOR_NOT_FOUND;
            break;
        case BLE_COM_E_BLE_LIB_NOT_LOADED:
            coines_ret = COINES_E_BLE_LIBRARY_NOT_LOADED;
            break;
        case BLE_COM_E_PERIPHERAL_NOT_FOUND:
            coines_ret = COINES_E_BLE_PERIPHERAL_NOT_FOUND;
            break;
        case BLE_COM_E_INVALID_COM_CONFIG:
            coines_ret = COINES_E_BLE_INVALID_CONFIG;
            break;
        case BLE_COM_E_ADAPTER_BLUETOOTH_NOT_ENABLED:
            coines_ret = COINES_E_BLE_ADAPTER_BLUETOOTH_NOT_ENABLED;
            break;
        case BLE_COM_E_APP_BOARD_NOT_FOUND:
            coines_ret = COINES_E_BLE_APP_BOARD_NOT_FOUND;
            break;
        default:
            coines_ret = COINES_E_BLE_COMM_FAILED;
    }

    return coines_ret;
}

/*!
 * @brief This API is used to format and send the packet
 *
 */
static int16_t coines_send_multi_packet(uint8_t command,
                                        uint8_t *payload_header,
                                        uint16_t header_length,
                                        uint8_t *payload_body,
                                        uint16_t body_length)
{
    int16_t ret = COM_OK;
    uint16_t bytes_to_write = 0;
    uint16_t packet_size_limit = 0;

    if ((header_length != 0) && (payload_header == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (comm_intf == COINES_COMM_INTF_USB)
    {
        packet_size_limit = COINES_PACKET_SIZE;
    }
    else if (comm_intf == COINES_COMM_INTF_BLE)
    {
        packet_size_limit = COINES_BLE_PACKET_SIZE;
    }

    cmd_packet_size = header_length + body_length + 4;

    cmd_packet[COINES_PROTO_HEADER_POS] = COINES_CMD_HEADER;
    memcpy(&cmd_packet[COINES_PROTO_LENGTH_POS], &cmd_packet_size, 2);
    cmd_packet[COINES_PROTO_CMD_POS] = command;
    if (header_length != 0)
    {
        memcpy(&cmd_packet[COINES_PROTO_PAYLOAD_POS], payload_header, header_length);
        if (payload_body && body_length)
        {
            memcpy(&cmd_packet[COINES_PROTO_PAYLOAD_POS + header_length], payload_body, body_length);
        }
    }

    for (uint16_t pi = 0; pi < cmd_packet_size; pi += bytes_to_write)
    {
        if (cmd_packet_size < packet_size_limit)
        {
            bytes_to_write = cmd_packet_size;
        }
        else if ((cmd_packet_size - pi) < packet_size_limit)
        {
            bytes_to_write = cmd_packet_size - pi;
        }
        else
        {
            bytes_to_write = packet_size_limit;
        }

        coines_write_intf(comm_intf, &cmd_packet[pi], bytes_to_write);

        if (com_write_status != COM_OK)
        {

            /* Assumes that the device recovers from the partial packet
             * with a timeout
             */
            return COINES_E_COMM_IO_ERROR;
        }
    }

    return ret;
}

/*!
 * @brief This API is used to format and send the packet
 *
 */
static int16_t coines_send_packet(uint8_t command, uint8_t *payload, uint16_t length)
{
    return coines_send_multi_packet(command, payload, length, NULL, 0);
}

/*!
 * @brief This API is used read and parse the received response
 *
 */
static int16_t read_data(uint8_t *read_buffer, uint16_t packet_length)
{
    uint32_t start_time;
    uint32_t current_time;
    uint16_t bytes_available;
    int16_t packet_idx = 0;

    start_time = coines_get_millis();
    do
    {
        bytes_available = coines_read_intf(comm_intf, &read_buffer[packet_idx], packet_length);
        if (com_read_status == COM_OK)
        {
            if (bytes_available)
            {
                packet_idx += (int16_t)bytes_available;
            }
        }
        else
        {
            return COINES_E_COMM_IO_ERROR;
        }
        current_time = coines_get_millis();
        if ((current_time - start_time) > READ_TIMEOUT_MS)
        {
            return COINES_E_READ_TIMEOUT;
        }
    } while (packet_idx < packet_length);

    return packet_idx;
}
/*!
 * @brief This API is used read and parse the received response
 *
 */
static int16_t coines_receive_resp(uint8_t command, uint16_t *resp_length)
{
    int16_t ret = COINES_SUCCESS;
    int16_t packet_idx;
    uint16_t packet_length = 0;
    uint16_t initial_read_len = 3;

    packet_idx = read_data(&resp_buffer[0], initial_read_len);
    if (packet_idx < 0)
    {
        /* Error occured */
        return packet_idx;
    }
    
    memcpy(&packet_length, &resp_buffer[COINES_PROTO_LENGTH_POS], 2);

    if (packet_length)
    {
        packet_idx = read_data(&resp_buffer[packet_idx], (uint16_t)(packet_length - packet_idx));
        if (packet_idx < 0) 
        {
            /* Error occured */
            return packet_idx;
        }

        if (resp_buffer[COINES_PROTO_HEADER_POS] == COINES_RESP_OK_HEADER)
        {
            *resp_length = packet_length - COINES_PROTO_PAYLOAD_POS;
            if (resp_buffer[COINES_PROTO_CMD_POS] != command)
            {
                return COINES_E_COMM_WRONG_RESPONSE;
            }
        }
        else if (resp_buffer[COINES_PROTO_HEADER_POS] == COINES_RESP_NOK_HEADER)
        {
            if (resp_buffer[COINES_PROTO_PAYLOAD_POS] != COINES_SUCCESS)
            {
                return (int8_t)resp_buffer[COINES_PROTO_PAYLOAD_POS];
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }
    }

    return ret;
}

/*!
 * @brief Calls BLE scan function
 */
int16_t coines_scan_ble_devices(struct ble_peripheral_info *ble_info, uint8_t *peripheral_count, uint32_t scan_timeout_ms)
{
    int8_t err_code = ble_scan(ble_info, peripheral_count, scan_timeout_ms);
    int16_t coines_ret = map_ble_error_codes(err_code);

    return coines_ret;
}

/*!
 * @brief This API is used to initialize the communication according to interface type.
 *
 */
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    struct coines_serial_com_config* scom_config;
    struct ble_peripheral_info* ble_config;

    comm_intf = intf_type;

    /* Allocate memory to resp_buffer */
    resp_buffer = (uint8_t*) malloc(COINES_BUFFER_SIZE * sizeof(uint8_t));
    switch (intf_type)
    {
        case COINES_COMM_INTF_USB:
            scom_config = (struct coines_serial_com_config*) arg;

            return coines_connect_usb(scom_config);
        case COINES_COMM_INTF_VCOM:

            return -1; /* TODO */
        case COINES_COMM_INTF_BLE:
            ble_config = (struct ble_peripheral_info*) arg;

            return coines_connect_ble(ble_config);
        default:
            break;
    }

    return COM_OK;
}

/*!
 * @brief This API is used to close the active communication(USB,COM or BLE).
 *
 */
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    (void)arg;
    switch (intf_type)
    {
        case COINES_COMM_INTF_USB:

            return coines_disconnect_usb();
        case COINES_COMM_INTF_VCOM:

            return -1; /* TODO */
        case COINES_COMM_INTF_BLE:

            return coines_disconnect_ble();
        default:
            break;
    }

    /* free allocated memory for resp_buffer */
    free(resp_buffer);

    return COM_OK;
}

/*!
 *  @brief This API is used to test the communication.
 *
 */
int16_t coines_echo_test(uint8_t *data, uint16_t length)
{
    int16_t ret;
    uint16_t resp_length = 0;

    ret = coines_send_packet(COINES_CMD_ID_ECHO, data, length);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_ECHO, &resp_length);
    }

    if ((ret == COINES_SUCCESS) && (memcmp(data, &resp_buffer[COINES_PROTO_PAYLOAD_POS], length)))
    {
        return COINES_E_COMM_WRONG_RESPONSE;
    }

    return ret;
}

/*!
 *  @brief This API is used to get the board information.
 *
 */
int16_t coines_get_board_info(struct coines_board_info *board_info)
{
    int16_t ret;
    uint16_t resp_length = 0;

    if (board_info == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    ret = coines_send_packet(COINES_CMD_ID_GET_BOARD_INFO, NULL, 0);

    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_GET_BOARD_INFO, &resp_length);
    }

    if (ret == COINES_SUCCESS)
    {
        memcpy(&board_info->hardware_id, &resp_buffer[COINES_PROTO_PAYLOAD_POS], 2);
        memcpy(&board_info->software_id, &resp_buffer[COINES_PROTO_PAYLOAD_POS + 2], 2);
        memcpy(&board_info->board, &resp_buffer[COINES_PROTO_PAYLOAD_POS + 4], 1);
        memcpy(&board_info->shuttle_id, &resp_buffer[COINES_PROTO_PAYLOAD_POS + 5], 2);
    }

    return ret;
}

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 *
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction direction,
                              enum coines_pin_value pin_value)
{
    uint8_t payload[3] = { pin_number, direction, pin_value };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = coines_send_packet(COINES_CMD_ID_SET_PIN, payload, 3);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SET_PIN, &resp_length);
    }

    (void)resp_length;

    return ret;
}

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 *
 */
int16_t coines_get_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction *pin_direction,
                              enum coines_pin_value *pin_value)
{
    uint8_t payload[3] = { pin_number, *pin_direction, *pin_value };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = coines_send_packet(COINES_CMD_ID_GET_PIN, payload, 3);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_GET_PIN, &resp_length);
    }

    if ((resp_length >= 3) && (pin_number == resp_buffer[COINES_PROTO_PAYLOAD_POS]))
    {
        *pin_direction = (enum coines_pin_direction)resp_buffer[COINES_PROTO_PAYLOAD_POS + 1];
        *pin_value = (enum coines_pin_value)resp_buffer[COINES_PROTO_PAYLOAD_POS + 2];
    }
    else
    {
        return COINES_E_COMM_WRONG_RESPONSE;
    }

    return ret;
}

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 *
 */
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
    uint8_t payload[4] = { 0 };
    int16_t ret;
    uint16_t resp_length = 0;

    memcpy(payload, &vdd_millivolt, 2);
    memcpy(&payload[2], &vddio_millivolt, 2);

    ret = coines_send_packet(COINES_CMD_ID_SET_VDD_VDDIO, payload, 4);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SET_VDD_VDDIO, &resp_length);
    }

    return ret;
}

/*!
 *  @brief This API is used to configure the spi bus
 *
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
{
    uint8_t payload[3] = { bus, spi_speed, spi_mode };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = coines_send_packet(COINES_CMD_ID_SPI_CONFIG, payload, 3);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SPI_CONFIG, &resp_length);
    }

    return ret;
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

    ret = coines_send_packet(COINES_CMD_ID_SPI_DECONFIG, &payload, 1);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SPI_DECONFIG, &resp_length);
    }

    return ret;
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
    uint8_t payload[4] = { bus, spi_speed, spi_mode, spi_transfer_bits };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = coines_send_packet(COINES_CMD_ID_SPI_WORD_CONFIG, payload, 4);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SPI_WORD_CONFIG, &resp_length);
    }

    return ret;
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

    ret = coines_send_packet(COINES_CMD_ID_I2C_CONFIG, payload, 2);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_I2C_CONFIG, &resp_length);
    }

    return ret;
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

    ret = coines_send_packet(COINES_CMD_ID_I2C_DECONFIG, &payload, 1);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_I2C_DECONFIG, &resp_length);
    }

    return ret;
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

    ret = coines_send_multi_packet(COINES_CMD_ID_I2C_WRITE_REG, payload, 5, reg_data, count);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_I2C_WRITE_REG, &resp_length);
    }

    return (int8_t)ret;
}

/*!
 *  @brief This API is used to write 16-bit register data on the I2C device.
 *
 */
int8_t coines_write_16bit_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    (void)bus;
    (void)dev_addr;
    (void)reg_addr;
    (void)reg_data;
    (void)count;
    (void)i2c_transfer_bits;

    return (int8_t)COINES_E_FAILURE;
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

    ret = coines_send_packet(COINES_CMD_ID_I2C_READ_REG, payload, 5);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_I2C_READ_REG, &resp_length);
        if (resp_length != count)
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }

        memcpy(reg_data, &resp_buffer[COINES_PROTO_PAYLOAD_POS], resp_length);
    }

    return (int8_t)ret;
}

/*!
 *  @brief This API is used to read 16-bit register data from the I2C device.
 *
 */
int8_t coines_read_16bit_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    (void)bus;
    (void)dev_addr;
    (void)reg_addr;
    (void)reg_data;
    (void)count;
    (void)i2c_transfer_bits;

    return (int8_t)COINES_E_FAILURE;
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 *
 */
int8_t coines_write_16bit_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    (void)bus;
    (void)cs_pin;
    (void)reg_addr;
    (void)reg_data;
    (void)count;
    (void)spi_transfer_bits;

    return (int8_t)COINES_E_FAILURE;
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

    ret = coines_send_multi_packet(COINES_CMD_ID_SPI_WRITE_REG, payload, 5, reg_data, count);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SPI_WRITE_REG, &resp_length);
    }

    return (int8_t)ret;
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 *
 */
int8_t coines_read_16bit_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    (void)bus;
    (void)cs_pin;
    (void)reg_addr;
    (void)reg_data;
    (void)count;
    (void)spi_transfer_bits;

    return (int8_t)COINES_E_FAILURE;
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

    ret = coines_send_packet(COINES_CMD_ID_SPI_READ_REG, payload, 5);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SPI_READ_REG, &resp_length);
        if (resp_length != count)
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }

        memcpy(reg_data, &resp_buffer[COINES_PROTO_PAYLOAD_POS], resp_length);
    }

    return (int8_t)ret;
}

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 *
 */
void coines_delay_msec(uint32_t delay_ms)
{
#ifdef PLATFORM_WINDOWS
    Sleep(delay_ms);
#else
    uint32_t delay_microsec = (uint32_t)(delay_ms * 1000);
    usleep(delay_microsec);
#endif
}

/*!
 *  @brief This API is used for introducing a delay in microseconds
 *
 */
void coines_delay_usec(uint32_t delay_us)
{
#ifdef PLATFORM_WINDOWS
    Sleep(delay_us < 1000 ? 1 : (delay_us / 1000));
#else
    usleep(delay_us);
#endif
}

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 *
 * @return Time in milliseconds
 */
uint32_t coines_get_millis(void)
{
    struct timeval current_time;
    uint32_t millisecond = 0;

    if (gettimeofday(&current_time, NULL) == 0)
    {
        millisecond = (uint32_t)((current_time.tv_sec * 1000) + (current_time.tv_usec / 1000));
    }

    return millisecond;
}

/*!
 * @brief This API returns the number of microseconds passed since the program started
 *
 */
uint64_t coines_get_micro_sec(void)
{
    struct timeval current_time;
    uint64_t microsecond = 0;

    if (gettimeofday(&current_time, NULL) == 0)
    {
        /*lint -e732 */
        microsecond = current_time.tv_usec;
    }

    return microsecond;
}

/*!
 * @brief Get COINES_SDK library version
 *
 * @return pointer to version string
 */
const char *coines_get_version()
{
    return (const char *)COINES_VERSION;
}

int16_t coines_read_temp_data(float *temp_data)
{
    (void)temp_data;

    return COINES_E_FAILURE;
}

/*!
 * @brief This API is used to open usb communication.
 *
 */
static int16_t coines_connect_usb(struct coines_serial_com_config *scom_config)
{
    int8_t scom_ret;
    int16_t coines_ret = COINES_SUCCESS;
    uint16_t new_rx_buffer_size;
    char com_port_name[COINES_CHAR_MAX_LEN];

    if (scom_config == NULL)
    {
        scom_ret = scom_open();
    }
    else
    {
        new_rx_buffer_size = scom_config->rx_buffer_size;
        strcpy(com_port_name, scom_config->com_port_name);
        #ifdef PLATFORM_WINDOWS
        if (strncmp(com_port_name, "COM", 3) != 0)
        #else
        if (strncmp(com_port_name, "/dev/", 5) != 0)
        #endif
        {
            return COINES_E_SCOM_INVALID_CONFIG;
        }

        if (new_rx_buffer_size > COINES_BUFFER_SIZE)
        {
            /* resize resp_buffer to user input */
            resp_buffer = (uint8_t*)realloc(resp_buffer, new_rx_buffer_size * sizeof(uint16_t));

        }

        scom_ret = scom_open_id(scom_config->baud_rate,
                                scom_config->vendor_id,
                                scom_config->product_id,
                                scom_config->com_port_name);
    }

    switch (scom_ret)
    {
        case SCOM_OK:
            coines_ret = COINES_SUCCESS;
            break;
        case SCOM_E_DEV_NOT_FOUND:
            coines_ret = COINES_E_DEVICE_NOT_FOUND;
            break;
        case SCOM_E_PORT_IN_USE:
            coines_ret = COINES_E_SCOM_PORT_IN_USE;
            break;
        case SCOM_E_PORT_NOT_OPEN:
            coines_ret = COINES_E_FAILURE;
            break;
        case SCOM_E_DEV_NOT_SERIAL_COM:
            coines_ret = COINES_E_INCOMPATIBLE_FIRMWARE;
            break;
        default:
            coines_ret = COINES_E_FAILURE;
    }

    return coines_ret;
}

/*!
 * @brief This API is used to open ble communication.
 *
 */
static int16_t coines_connect_ble(struct ble_peripheral_info *ble_config)
{
    int8_t ble_com_ret;
    int16_t coines_ret;

    ble_com_ret = ble_connect(ble_config);

    coines_ret = map_ble_error_codes(ble_com_ret);

    return coines_ret;
}

/*!
 * @brief This API is used to close usb communication.
 *
 */
static int16_t coines_disconnect_usb(void)
{
    int8_t scom_ret;
    int16_t coines_ret = COINES_SUCCESS;

    scom_ret = scom_close();
    switch (scom_ret)
    {
        case SCOM_OK:
            coines_ret = COINES_SUCCESS;
            break;
        case SCOM_E_DEV_NOT_FOUND:
            coines_ret = COINES_E_DEVICE_NOT_FOUND;
            break;
        case SCOM_E_PORT_IN_USE:
            coines_ret = COINES_E_UNABLE_CLAIM_INTF;
            break;
        case SCOM_E_PORT_NOT_OPEN:
            coines_ret = COINES_E_FAILURE;
            break;
        default:
            coines_ret = COINES_E_FAILURE;
    }

    return coines_ret;
}

/*!
 * @brief This API is used to close BLE communication.
 *
 */
static int16_t coines_disconnect_ble(void)
{
    int8_t ble_com_ret;
    int16_t coines_ret;

    ble_com_ret = ble_close();
    coines_ret = map_ble_error_codes(ble_com_ret);

    return coines_ret;
}

/*!
 * @brief This API is used to configure streaming settings.
 *
 */
int16_t coines_config_streaming(uint8_t sensor_id,
                                struct coines_streaming_config *stream_config,
                                struct coines_streaming_blocks *data_blocks)
{
    int16_t rslt = COINES_SUCCESS;
    struct coines_streaming_settings *stream_p;

    if ((stream_config != NULL) && (data_blocks != NULL))
    {
        if (coines_sensor_id_count < COINES_MAX_SENSOR_ID)
        {
            stream_p = (struct coines_streaming_settings*)&coines_streaming_cfg_buf[coines_sensor_id_count];
            memset(stream_p, 0, sizeof(struct coines_streaming_settings));

            stream_p->sensor_id = sensor_id;
            memcpy(&stream_p->stream_config, stream_config, sizeof(struct coines_streaming_config));
            memcpy(&stream_p->data_blocks, data_blocks, sizeof(struct coines_streaming_blocks));
            if (stream_p->data_blocks.no_of_blocks == 0 || stream_p->data_blocks.no_of_blocks > COINES_MAX_BLOCKS)
            {
                return COINES_E_STREAM_INVALID_BLOCK_SIZE;
            }

            coines_sensor_id_count++;
        }
        else
        {
            return COINES_E_MAX_SENSOR_COUNT_REACHED;
        }
    }
    else
    {
        rslt = COINES_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to configure polling streaming sample time.
 *
 */
static int16_t config_poll_stream_timing()
{
    int16_t ret;
    double sampling_time[2] = { 0, 0 };
    double remaining = 0;
    uint16_t resp_length = 0;
    uint8_t sampling_unit[2];
    uint16_t gcd_sampling_time;
    enum coines_sampling_unit gcd_sampling_unit;
    uint32_t i;

    /*check if sensor id count is greater than 1*/
    if (coines_sensor_id_count > 1)
    {
        for (i = 0; i < coines_sensor_id_count; i++)
        {
            sampling_time[i] = (double)coines_streaming_cfg_buf[i].stream_config.sampling_time;
            sampling_unit[i] = coines_streaming_cfg_buf[i].stream_config.sampling_units;
            sampling_time[i] =
                (sampling_unit[i] ==
                    COINES_SAMPLING_TIME_IN_MICRO_SEC) ? (sampling_time[i] / 1000.00) : sampling_time[i];
        }

        /* Calculate GCD */
        while (sampling_time[1] != 0)
        {
            remaining = (double)fmod(sampling_time[0], sampling_time[1]);
            sampling_time[0] = sampling_time[1];
            sampling_time[1] = remaining;
        }

        /* If decimal point is present, convert to microsecond */
        if ((sampling_time[0] - (int32_t)sampling_time[0]) != 0)
        {
            /* Need to convert to microsecond */
            gcd_sampling_time = (uint16_t)(sampling_time[0] * 1000);
            gcd_sampling_unit = COINES_SAMPLING_TIME_IN_MICRO_SEC;
        }
        else
        {
            gcd_sampling_time = (uint16_t)sampling_time[0];
            gcd_sampling_unit = COINES_SAMPLING_TIME_IN_MILLI_SEC;
        }
    }
    else
    {
        gcd_sampling_time = coines_streaming_cfg_buf[coines_sensor_id_count - 1].stream_config.sampling_time;
        gcd_sampling_unit = coines_streaming_cfg_buf[coines_sensor_id_count - 1].stream_config.sampling_units;
    }

    /*general streaming settings*/
    uint8_t payload[COINES_POLL_STREAM_COMMON_PAYLOAD_LEN];
    payload[0] = coines_sensor_id_count;
    payload[1] = gcd_sampling_time >> 8;
    payload[2] = gcd_sampling_time & 0xFF;
    payload[3] = gcd_sampling_unit;

    ret = coines_send_packet(COINES_CMD_ID_POLL_STREAM_COMMON, payload, COINES_POLL_STREAM_COMMON_PAYLOAD_LEN);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_POLL_STREAM_COMMON, &resp_length);
    }
    else
    {
        return ret;
    }
   
    return ret;
}

static int16_t dma_stream_config(void)
{
    int16_t ret = COINES_SUCCESS;
    uint16_t no_of_bytes_read = 0;
    uint8_t write_index = 0;
    uint16_t payload_len = 0;
    uint16_t resp_length;
    uint8_t payload[COINES_STREAM_CONFIG_BUFF_SIZE];

    for (uint8_t i = 0; i < coines_sensor_id_count; i++)
    {
        memset(&payload[0], 0x0, sizeof(payload));
        payload[write_index++] = coines_streaming_cfg_buf[i].sensor_id;
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.int_timestamp;
        payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.intf;
        if (coines_streaming_cfg_buf[i].stream_config.intf == COINES_SENSOR_INTF_I2C)
        {
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.i2c_bus;
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dev_addr;
        }
        else /* COINES_SENSOR_INTF_SPI */
        {
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.spi_bus;
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.cs_pin;
        }

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.int_pin;

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dma_config.ctlr_addr >> 8;
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dma_config.ctlr_addr & 0xFF;

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dma_config.startaddr_cmd >> 8;
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dma_config.startaddr_cmd & 0xFF;

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dma_config.read_addr >> 8;
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dma_config.read_addr & 0xFF;

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dma_config.read_len;

            
        no_of_bytes_read += coines_streaming_cfg_buf[i].stream_config.dma_config.read_len;
        
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.spi_type;
        
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.hw_pin_state;

        no_of_bytes_read += 10; /* if extra bytes in resp packet, timestamp(6bytes) and packet no(4bytes) */
        
        coines_sensor_info.sensors_byte_count[i] = no_of_bytes_read;

        payload_len = write_index;

        ret = coines_send_packet(COINES_CMD_ID_DMA_INT_STREAM_CONFIG, &payload[0], payload_len);
        if (ret == COINES_SUCCESS)
        {
            ret = coines_receive_resp(COINES_CMD_ID_DMA_INT_STREAM_CONFIG, &resp_length);
            if (ret != COINES_SUCCESS)
            {
                return ret;
            }
        }
        else
        {
            return ret;
        }

        write_index = 0;
        payload_len = 0;
        no_of_bytes_read = 0;
    }
    
    coines_sensor_info.stream_mode = COINES_STREAMING_MODE_DMA_INTERRUPT;
    return ret;

}

/*!
 * @brief This API is used to configure polling streaming.
 *
 */
static int16_t poll_stream_config(void)
{
    int16_t ret;
    uint16_t no_of_bytes_read = 0;
    uint8_t write_index = 0;
    uint16_t payload_len = 0;
    uint16_t resp_length;
    uint8_t payload[COINES_STREAM_CONFIG_BUFF_SIZE];

    ret = config_poll_stream_timing();

    if (ret == COINES_SUCCESS)
    {
        for (uint8_t i = 0; i < coines_sensor_id_count; i++)
        {
            memset(&payload[0], 0x0, sizeof(payload));
            payload[write_index++] = coines_streaming_cfg_buf[i].sensor_id;
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.int_timestamp;
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.intf;
            if (coines_streaming_cfg_buf[i].stream_config.intf == COINES_SENSOR_INTF_I2C)
            {
                payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.i2c_bus;
                payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dev_addr;
            }
            else /* COINES_SENSOR_INTF_SPI */
            {
                payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.spi_bus;
                payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.cs_pin;
            }

            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.sampling_time >> 8;
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.sampling_time & 0xFF;
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.sampling_units;

            no_of_bytes_read += 0; /* if extra bytes in resp packet */
            
            payload[write_index++] = coines_streaming_cfg_buf[i].data_blocks.no_of_blocks >> 8;
            payload[write_index++] = coines_streaming_cfg_buf[i].data_blocks.no_of_blocks & 0xFF;

            for (uint16_t j = 0; j < coines_streaming_cfg_buf[i].data_blocks.no_of_blocks; j++)
            {
                payload[write_index++] = coines_streaming_cfg_buf[i].data_blocks.reg_start_addr[j];
                payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].data_blocks.no_of_data_bytes[j];
                no_of_bytes_read += coines_streaming_cfg_buf[i].data_blocks.no_of_data_bytes[j];
            }

            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.spi_type;
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.clear_on_write;

            if (coines_streaming_cfg_buf[i].stream_config.clear_on_write)
            {
                payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.clear_on_write_config.dummy_byte;
                payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.clear_on_write_config.startaddress;
                payload[write_index++] =
                    (uint8_t)coines_streaming_cfg_buf[i].stream_config.clear_on_write_config.num_bytes_to_clear;
            }

            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.intline_count;

            for (uint16_t j = 0; j < coines_streaming_cfg_buf[i].stream_config.intline_count; j++)
            {
                payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.intline_info[j];
            }

            coines_sensor_info.sensors_byte_count[i] = no_of_bytes_read;

            payload_len = write_index;

            ret = coines_send_packet(COINES_CMD_ID_POLL_STREAM_CONFIG, &payload[0], payload_len);
            if (ret == COINES_SUCCESS)
            {
                ret = coines_receive_resp(COINES_CMD_ID_POLL_STREAM_CONFIG, &resp_length);
                if (ret != COINES_SUCCESS)
                {
                    return ret;
                }
            }
            else
            {
                return ret;
            }

            write_index = 0;
            payload_len = 0;
            no_of_bytes_read = 0;
        }
    }        
    
    return ret;
}

/*!
 * @brief This API is used to configure interrupt streaming.
 *
 */
static int16_t interrupt_stream_config(void)
{
    int16_t ret = COINES_SUCCESS;
    uint16_t no_of_bytes_read = 0;
    uint8_t write_index = 0;
    uint16_t payload_len = 0;
    uint16_t resp_length;
    uint8_t payload[COINES_STREAM_CONFIG_BUFF_SIZE];

    for (uint8_t i = 0; i < coines_sensor_id_count; i++)
    {
        memset(&payload[0], 0x0, sizeof(payload));
        payload[write_index++] = coines_streaming_cfg_buf[i].sensor_id;
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.int_timestamp;
        payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.intf;
        if (coines_streaming_cfg_buf[i].stream_config.intf == COINES_SENSOR_INTF_I2C)
        {
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.i2c_bus;
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.dev_addr;
        }
        else /* COINES_SENSOR_INTF_SPI */
        {
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.spi_bus;
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].stream_config.cs_pin;
        }

        
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.int_pin;

        no_of_bytes_read += 10; /* if extra bytes in resp packet, timestamp(6bytes) and packet no(4bytes) */
       
        payload[write_index++] = coines_streaming_cfg_buf[i].data_blocks.no_of_blocks >> 8;
        payload[write_index++] = coines_streaming_cfg_buf[i].data_blocks.no_of_blocks & 0xFF;

        for (uint16_t j = 0; j < coines_streaming_cfg_buf[i].data_blocks.no_of_blocks; j++)
        {
            payload[write_index++] = coines_streaming_cfg_buf[i].data_blocks.reg_start_addr[j];
            payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[i].data_blocks.no_of_data_bytes[j];
            no_of_bytes_read += coines_streaming_cfg_buf[i].data_blocks.no_of_data_bytes[j];
        }

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.spi_type;
        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.clear_on_write;

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.hw_pin_state;

        if (coines_streaming_cfg_buf[i].stream_config.clear_on_write)
        {
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.clear_on_write_config.dummy_byte;
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.clear_on_write_config.startaddress;
            payload[write_index++] =
                (uint8_t)coines_streaming_cfg_buf[i].stream_config.clear_on_write_config.num_bytes_to_clear;
        }

        payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.intline_count;

        for (uint16_t j = 0; j < coines_streaming_cfg_buf[i].stream_config.intline_count; j++)
        {
            payload[write_index++] = coines_streaming_cfg_buf[i].stream_config.intline_info[j];
        }

        coines_sensor_info.sensors_byte_count[i] = no_of_bytes_read;

        payload_len = write_index;

        ret = coines_send_packet(COINES_CMD_ID_INT_STREAM_CONFIG, &payload[0], payload_len);
        if (ret == COINES_SUCCESS)
        {
            ret = coines_receive_resp(COINES_CMD_ID_INT_STREAM_CONFIG, &resp_length);
            if (ret != COINES_SUCCESS)
            {
                return ret;
            }
        }
        else
        {
            return ret;
        }

        write_index = 0;
        payload_len = 0;
        no_of_bytes_read = 0;
    }
    return ret;
}

/*!
 * @brief This API is used to send streaming setting and start/stop the streaming.
 *
 */
int16_t coines_start_stop_streaming(enum coines_streaming_mode stream_mode, uint8_t start_stop)
{
    int16_t ret = COINES_SUCCESS;
    uint32_t i;
    uint16_t payload_len = 0;
    uint16_t resp_length;
    uint8_t payload[COINES_STREAM_CONFIG_BUFF_SIZE];
 
    /*! Holds the current time as a time_t value */
    time_t system_time; 

    /*! Pointer to a tm structure holding the decomposed time */
    struct tm *tm_local;

    /*check the if it is start request for polling streaming*/
    if (start_stop)
    {
        coines_sensor_info.no_of_sensors_enabled = coines_sensor_id_count;

        if (stream_mode == COINES_STREAMING_MODE_DMA_INTERRUPT)
        {
            (void)dma_stream_config();
        }
        else if (stream_mode == COINES_STREAMING_MODE_POLLING)
        {
            (void)poll_stream_config();
        }
        else if(stream_mode == COINES_STREAMING_MODE_INTERRUPT)
        {
            (void)interrupt_stream_config();
        }
        
        if(start_stop == COINES_USB_STREAMING_START)
        {
            // Enable USB streaming
            payload[0] = 0xFF;
            payload_len = 1;
            ret = coines_send_packet(COINES_CMD_ID_STREAM_START_STOP, &payload[0], payload_len);
        }
        else if (start_stop == COINES_EXT_FLASH_STREAMING_START)
        {
            // Enable external flash logging 
            payload[0] = 0xFE;
            /*lint -e64*/
            (void)time(&system_time);
            tm_local = localtime(&system_time);
            /*lint +e64*/
            
            // Format the current date and time as a string ("YYYY-MM-DD HH:MM:SS")
            (void)strftime(datetime_str, sizeof(datetime_str), "StreamData_%Y-%m-%d_%H:%M:%S", tm_local);
            datetime_str_len = (uint16_t)strlen(datetime_str) + 1; // 1 byte for null terminator
            payload_len = datetime_str_len + 1; // 1 byte for the command

            memcpy(&payload[1], datetime_str, datetime_str_len);
            ret = coines_send_packet(COINES_CMD_ID_STREAM_START_STOP, &payload[0], payload_len);
        }
       
        
        if (ret == COINES_SUCCESS)
        {
            ret = coines_receive_resp(COINES_CMD_ID_STREAM_START_STOP, &resp_length);
        }
    }
    else
    {
        memset(&payload[0], 0x0, sizeof(payload));
        ret = coines_send_packet(COINES_CMD_ID_STREAM_START_STOP, &payload[0], 1);
        if (ret == COINES_SUCCESS)
        {
            /* Read untill stop streaming rsp is found */
            do
            {
                ret = coines_receive_resp(COINES_CMD_ID_STREAM_START_STOP, &resp_length);
            } while (ret != COINES_SUCCESS);
        }

        /* clean-up */
        for (i = 0; i < coines_sensor_id_count; i++)
        {
            memset(&coines_streaming_cfg_buf[i], 0, sizeof(struct coines_streaming_settings));
        }
       
        coines_sensor_id_count = 0;
    }

    return ret;
}

/*!
 * @brief This API is used to read the streaming sensor data.
 *
 */
int16_t coines_read_stream_sensor_data(uint8_t sensor_id,
                                       uint32_t number_of_samples,
                                       uint8_t *data,
                                       uint32_t *valid_samples_count)
{
    int16_t ret = COINES_SUCCESS;
    uint16_t resp_length = 0;
    uint16_t no_of_bytes = coines_sensor_info.sensors_byte_count[sensor_id - 1];
    uint16_t write_index = 0;
    uint16_t samples = 0;

    if ((data == NULL) || (valid_samples_count == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    /*TODO - Instead of waiting in the loop, should immediatly return with the number of available samples*/
    for (uint16_t i = 0; i < 65535; i++)
    {
        memset(&resp_buffer[5], 0, no_of_bytes);
        ret = coines_receive_resp(COINES_READ_SENSOR_DATA, &resp_length);
        if (ret == COINES_SUCCESS)
        {
            if (resp_buffer[4] == sensor_id)
            {
                memcpy((data + write_index), &resp_buffer[5], no_of_bytes);
                write_index += no_of_bytes;
                if (++samples == number_of_samples)
                {
                    break;
                }
            }
        }
        else
        {
            return ret;
        }
    }

    *valid_samples_count = samples;

    return ret;
}

/*!
 * @brief This API is used to trigger the soft reset.
 *
 */
void coines_soft_reset(void)
{
    uint8_t payload[4];
    int16_t ret;
    uint16_t resp_length = 0;

    payload[0] = 0x00;
    payload[1] = 0x00;
    payload[2] = 0x0F;
    payload[3] = 0x00;

    ret = coines_send_packet(COINES_CMD_ID_SOFT_RESET, payload, 4);
    if (ret == COINES_SUCCESS)
    {
        coines_receive_resp(COINES_CMD_ID_SOFT_RESET, &resp_length);
    }
}

/*!
 * @brief This API is used to read data over the specified interface
 *
 */
uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    uint32_t n_bytes_read = 0;

    if (intf == COINES_COMM_INTF_USB)
    {
        com_read_status = scom_read(buffer, (uint32_t)len, &n_bytes_read);
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        com_read_status = ble_read(buffer, (uint32_t)len, &n_bytes_read);
    }

    return (uint16_t)n_bytes_read;
}

/*!
 * @brief This API is used write data over the specified interface
 *
 */
void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        com_write_status = scom_write(buffer, (uint32_t)len);
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        com_write_status = ble_write(buffer, (uint32_t)len);
    }
}

/*!
 * @brief This API is used to flush the buffer
 *
 */
void coines_flush_intf(enum coines_comm_intf intf)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        /*lint -e534*/
        scom_clear_buffer();

        /*lint +e534*/
    }
}

int16_t coines_trigger_timer(enum coines_timer_config tmr_cfg, enum coines_time_stamp_config ts_cfg)
{
    (void)tmr_cfg;
    (void)ts_cfg;

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to write the content into shuttle eeprom
 */
int16_t coines_shuttle_eeprom_write(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{
    int16_t ret;
    uint16_t resp_length = 0;

    ret = coines_send_multi_packet(COINES_CMD_ID_SHUTTLE_EEPROM_WRITE, (uint8_t *)&start_addr, 1, buffer, length);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SHUTTLE_EEPROM_WRITE, &resp_length);
    }

    return ret;
}

/*!
 * @brief This API is used to read the content from the shuttle eeprom
 */
int16_t coines_shuttle_eeprom_read(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{
    int16_t ret;
    uint16_t resp_length = 0;

    ret = coines_send_packet(COINES_CMD_ID_SHUTTLE_EEPROM_READ, (uint8_t *)&start_addr, length);
    if (ret == COINES_SUCCESS)
    {
        ret = coines_receive_resp(COINES_CMD_ID_SHUTTLE_EEPROM_READ, &resp_length);
        if (ret == COINES_SUCCESS)
        {
            memcpy(buffer, &resp_buffer[COINES_PROTO_PAYLOAD_POS], length);
        }
    }

    return ret;
}
