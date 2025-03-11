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
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "coines_bridge_firmware.h"
#include "coines_bridge_client.h"
#include "coines_bridge_stream.h"
#include "decoder.h"
#include "stream.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define COINES_COMM_INTF_SERIAL  COINES_COMM_INTF_USB

#if (defined(MCU_APP30)||defined(MCU_APP31))
#define LED_ACTIVITY_INDICATOR
#endif

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/*! Communication interface used by COINES bridge firmware. */
enum coines_comm_intf comm_intf = COINES_COMM_INTF_BLE;

/*! Timeout for data transfer operations. */
uint64_t transfer_timeout;

/*! Flag to enable or disable logging to external flash. */
bool ext_flash_log_enabled;

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
/*! LED timeouts for different condition */
static uint32_t life_led_error_period = 500;
static uint32_t life_led_normal_period = 2500;
static uint32_t life_led_blink_period = 50;

/*! Updated when soft reset is triggered */
static bool soft_reset_triggered = false;

/*! Buffer used to hold the read data */
static uint8_t packet[READ_BUFF_SIZE] = { 0 };

/*! Buffer used to hold the write data */
static uint8_t resp_buff[WRITE_BUFF_SIZE] = { 0 };

static bool error = false;

static enum coines_led led = COINES_LED_GREEN;

static bool intf_connected = false;

static uint32_t blink_on = 0, blink_off = 0;
static bool led_state = false;
static bool read_buff_overflow = false;
static uint16_t packet_length = 0;
static uint16_t rsp_length;
struct coines_cbt cbt = { 0 };

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/
static int16_t echo_callback(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t get_board_info_callback(uint8_t cmd,
                                       uint8_t *payload,
                                       uint16_t payload_length,
                                       uint8_t *resp,
                                       uint16_t *resp_length);
static int16_t set_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length);
static int16_t get_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length);
static int16_t set_vdd_vddio(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t spi_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length);
static int16_t spi_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t i2c_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length);
static int16_t i2c_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t i2c_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t i2c_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t spi_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t spi_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
#if defined(MCU_APP30) || defined(MCU_APP31)
static int16_t spi_write_16bit_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t spi_read_16bit_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t i2c_write_16bit_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t i2c_read_16bit_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
#endif

static int16_t soft_reset(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length);
static int16_t shuttle_eeprom_write(uint8_t cmd,
                                    uint8_t *payload,
                                    uint16_t payload_length,
                                    uint8_t *resp,
                                    uint16_t *resp_length);
static int16_t shuttle_eeprom_read(uint8_t cmd,
                                   uint8_t *payload,
                                   uint16_t payload_length,
                                   uint8_t *resp,
                                   uint16_t *resp_length);
static void update_intf(void);
static void process_packet(void);
static void blink_led(void);

/**********************************************************************************/
/* functions */
/**********************************************************************************/
/*!
 *  @brief This API is used to check and indicate interface connection via LED
 *
 */
static void update_intf(void)
{
    if (coines_intf_connected(COINES_COMM_INTF_BLE))
    {
        comm_intf = COINES_COMM_INTF_BLE;
        led = COINES_LED_BLUE;
        intf_connected = true;
    }
    else
    {
        comm_intf = COINES_COMM_INTF_SERIAL;
        led = COINES_LED_GREEN;
        intf_connected = true;
    }
}

/*!
 *  @brief This API is used to process read packet
 *
 */
static void process_packet(void)
{
    int8_t cmd_resp = COINES_SUCCESS;
#ifdef LED_ACTIVITY_INDICATOR
    coines_set_led(led, COINES_LED_STATE_ON); /* Turn on the LED to indicate command is being processed */
#endif

    coines_read_intf(comm_intf, packet, 3);
    if (packet[COINES_PROTO_HEADER_POS] == COINES_CMD_HEADER)
    {
        packet_length = 0;
        rsp_length = 0;
        memcpy(&packet_length, &packet[COINES_PROTO_LENGTH_POS], 2);

        if (coines_intf_available(comm_intf) == (READ_BUFF_SIZE - 1) - 3)
        {
            coines_flush_intf(comm_intf);
            read_buff_overflow = true;
        }
        else
        {
            while (coines_intf_available(comm_intf) < (packet_length - 3))
                ;

            coines_read_intf(comm_intf, &packet[COINES_PROTO_CMD_POS], packet_length - 3);

            if (packet_length >= 3)
            {
                cmd_resp = coines_process_packet(packet, packet_length, resp_buff, &rsp_length, &cbt);
                if (cmd_resp != COINES_SUCCESS)
                {
                    resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_NOK_HEADER;
                    resp_buff[COINES_PROTO_CMD_POS] = packet[COINES_PROTO_CMD_POS];
                    rsp_length = 5;
                    memcpy(&resp_buff[COINES_PROTO_LENGTH_POS], &rsp_length, 2);
                    resp_buff[COINES_PROTO_PAYLOAD_POS] = (uint8_t)cmd_resp;
                    error = true;
                    life_led_error_period = 1000;
                }

                if (rsp_length)
                {
                    coines_write_intf(comm_intf, resp_buff, rsp_length);
                    rsp_length = 0;
                }
            }
        }

        if (read_buff_overflow)
        {
            resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_NOK_HEADER;
            resp_buff[COINES_PROTO_CMD_POS] = packet[COINES_PROTO_CMD_POS];
            rsp_length = 5;
            memcpy(&resp_buff[COINES_PROTO_LENGTH_POS], &rsp_length, 2);
            resp_buff[COINES_PROTO_PAYLOAD_POS] = (uint8_t)COINES_E_MEMORY_ALLOCATION;
            coines_write_intf(comm_intf, resp_buff, rsp_length);
            rsp_length = 0;
            read_buff_overflow = false;
        }
    }
    else if ((packet[DECODER_BYTEPOS_HEADER] == DECODER_HEADER_VALUE) ||
             (packet[DECODER_BYTEPOS_HEADER] == UINT8_C(0x55)))
    {

#ifdef LED_ACTIVITY_INDICATOR
        coines_set_led(led, COINES_LED_STATE_ON);
#endif

        packet_length = packet[DECODER_BYTEPOS_SIZE];
        if (coines_intf_available(comm_intf) == (READ_BUFF_SIZE - 1) - 3)
        {
            coines_flush_intf(comm_intf);
            read_buff_overflow = true;
        }
        else
        {

            while (coines_intf_available(comm_intf) < (packet_length - 3))
                ;

            coines_read_intf(comm_intf, &packet[COINES_PROTO_CMD_POS], packet_length - 3);

            decoder_process_cmds(packet);

        }

#ifdef LED_ACTIVITY_INDICATOR
        coines_set_led(led, COINES_LED_STATE_OFF);
#endif

        if (read_buff_overflow)
        {
            resp_buff[0] = DECODER_HEADER_VALUE;
            resp_buff[1] = 8;
            resp_buff[2] = 1;
            resp_buff[3] = (uint8_t) COINES_E_MEMORY_ALLOCATION;
            resp_buff[4] = packet[DECODER_BYTEPOS_DIR] | DECODER_RSP_ID_MASK;
            resp_buff[5] = packet[DECODER_BYTEPOS_FEATURE];
            resp_buff[6] = DECODER_CR_MACRO;
            resp_buff[7] = DECODER_LF_MACRO;

            coines_write_intf(comm_intf, resp_buff, 8);
            read_buff_overflow = false;
        }
    }
    else
    {
        error = true;
        life_led_error_period = 125;
    }

#ifdef LED_ACTIVITY_INDICATOR
    coines_set_led(led, COINES_LED_STATE_OFF); /* Turn off the LED to indicate processing is done */
#endif
}

/*!
 *  @brief This API is used to blink led to indicate board connection and process completion
 *
 */
static void blink_led(void)
{

    if (blink_on <= coines_get_millis())
    {
        if (error)
        {
            blink_on = coines_get_millis() + life_led_error_period;
        }
        else
        {
            blink_on = coines_get_millis() + life_led_normal_period;
        }

        blink_off = coines_get_millis() + life_led_blink_period;
        coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON); /* Turn on the LED to indicate that the board is
                                                                * connected */
        led_state = true;
    }

    if (led_state && (blink_off <= coines_get_millis()))
    {
        coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF); /* Turn off the LED to indicate processing is done
                                                                */
        led_state = false;
    }
}

int main(void)
{

    uint32_t bytes_available = 0;

#if defined(MCU_APP30) || defined(MCU_APP31) || defined(MCU_HEAR3X)
    /* Open communication interface and wait till USB is connected */ 
    coines_open_comm_intf(comm_intf, NULL); 
#elif defined(MCU_NICLA)
    /* Configuration for NICLA MCU */
    struct coines_comm_intf_config intfconfig =
	{
		.uart_baud_rate = COINES_UART_BAUD_RATE_9600
	};
    coines_open_comm_intf(comm_intf, &intfconfig); 
#endif

    transfer_timeout = coines_get_millis();

    #ifdef DEBUG
    coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_6, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_5, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    #endif
    
    coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF); /* Turn off the LED to indicate that the board is connected */
    cbt.cmd_callback[COINES_CMD_ID_ECHO] = (coines_cmd_callback)echo_callback;
    cbt.cmd_callback[COINES_CMD_ID_GET_BOARD_INFO] = (coines_cmd_callback)get_board_info_callback;
    cbt.cmd_callback[COINES_CMD_ID_SET_PIN] = (coines_cmd_callback)set_pin_callback;
    cbt.cmd_callback[COINES_CMD_ID_GET_PIN] = (coines_cmd_callback)get_pin_callback;
    cbt.cmd_callback[COINES_CMD_ID_SET_VDD_VDDIO] = (coines_cmd_callback)set_vdd_vddio;
    cbt.cmd_callback[COINES_CMD_ID_SPI_CONFIG] = (coines_cmd_callback)spi_config;
    cbt.cmd_callback[COINES_CMD_ID_SPI_DECONFIG] = (coines_cmd_callback)spi_deconfig;
    cbt.cmd_callback[COINES_CMD_ID_I2C_CONFIG] = (coines_cmd_callback)i2c_config;
    cbt.cmd_callback[COINES_CMD_ID_I2C_DECONFIG] = (coines_cmd_callback)i2c_deconfig;
    cbt.cmd_callback[COINES_CMD_ID_I2C_WRITE_REG] = (coines_cmd_callback)i2c_write_reg;
    cbt.cmd_callback[COINES_CMD_ID_I2C_READ_REG] = (coines_cmd_callback)i2c_read_reg;
    cbt.cmd_callback[COINES_CMD_ID_SPI_WRITE_REG] = (coines_cmd_callback)spi_write_reg;
    cbt.cmd_callback[COINES_CMD_ID_SPI_READ_REG] = (coines_cmd_callback)spi_read_reg;
#if defined(MCU_APP30) || defined(MCU_APP31)
    cbt.cmd_callback[COINES_CMD_ID_SPI_WRITE_REG_16] = (coines_cmd_callback)spi_write_16bit_reg;
    cbt.cmd_callback[COINES_CMD_ID_SPI_READ_REG_16] = (coines_cmd_callback)spi_read_16bit_reg;
    cbt.cmd_callback[COINES_CMD_ID_I2C_WRITE_REG_16] = (coines_cmd_callback)i2c_write_16bit_reg;
    cbt.cmd_callback[COINES_CMD_ID_I2C_READ_REG_16] = (coines_cmd_callback)i2c_read_16bit_reg;
#endif
    cbt.cmd_callback[COINES_CMD_ID_POLL_STREAM_COMMON] = (coines_cmd_callback)poll_streaming_common;
    cbt.cmd_callback[COINES_CMD_ID_POLL_STREAM_CONFIG] = (coines_cmd_callback)poll_streaming_config;
    cbt.cmd_callback[COINES_CMD_ID_INT_STREAM_CONFIG] = (coines_cmd_callback)int_streaming_config;
    cbt.cmd_callback[COINES_CMD_ID_STREAM_START_STOP] = (coines_cmd_callback)streaming_start_stop;
    cbt.cmd_callback[COINES_CMD_ID_SOFT_RESET] = (coines_cmd_callback)soft_reset;
    cbt.cmd_callback[COINES_CMD_ID_SHUTTLE_EEPROM_WRITE] = (coines_cmd_callback)shuttle_eeprom_write;
    cbt.cmd_callback[COINES_CMD_ID_SHUTTLE_EEPROM_READ] = (coines_cmd_callback)shuttle_eeprom_read;
    cbt.cmd_callback[COINES_CMD_ID_DMA_INT_STREAM_CONFIG] = (coines_cmd_callback)dma_int_streaming_config;
    
    while (1)
    {
        update_intf();

        if (intf_connected)
        {
            bytes_available = coines_intf_available(comm_intf);
            if (bytes_available >= 3)
            {
#ifdef LED_ACTIVITY_INDICATOR
                coines_set_led(led, COINES_LED_STATE_ON); /* Turn on the LED to indicate command is being processed */
#endif
                process_packet();
            }

            blink_led();

            send_streaming_response(); /* Look for configured streaming and send the streaming response */
            send_legacy_protocol_streaming_response(); /* Look for configured streaming in the old-protocol and send the
                                                 * response */

            if (soft_reset_triggered)
            {
                coines_close_comm_intf(comm_intf, NULL);
                coines_delay_msec(200);
                coines_soft_reset();
            }

            transmit_streaming_rsp();

        }
    }
}

/*!
 *  @brief This API is used to transmit the data to PC.
 *
 */
void write_resp(void *buff, uint16_t len)
{
    coines_write_intf(comm_intf, buff, len);
}

/*!
 *  @brief This API is used to echo the received data.
 *
 */
static int16_t echo_callback(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = payload_length + 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;
    memcpy(&resp[COINES_PROTO_PAYLOAD_POS], payload, payload_length);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to get the board information.
 *
 */
static int16_t get_board_info_callback(uint8_t cmd,
                                       uint8_t *payload,
                                       uint16_t payload_length,
                                       uint8_t *resp,
                                       uint16_t *resp_length)
{
    (void)payload_length;
    struct coines_board_info board_info = { 0 };
    int16_t rslt;
    uint8_t read_index = 0;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    rslt = coines_get_board_info(&board_info);
    if (rslt == COINES_SUCCESS)
    {
        read_index = COINES_PROTO_PAYLOAD_POS;
        resp[read_index++] = (uint8_t) (board_info.hardware_id);
        resp[read_index++] = (uint8_t) (board_info.hardware_id >> 8);
        resp[read_index++] = (uint8_t) ((SOFTWARE_VERSION_MINOR & 0x0F) << 4) | (SOFTWARE_VERSION_PATCH & 0x0F);
        resp[read_index++] = (uint8_t) (SOFTWARE_VERSION_MAJOR & 0xFF);
        resp[read_index++] = (uint8_t) (board_info.board);
        resp[read_index++] = (uint8_t) (board_info.shuttle_id);
        resp[read_index++] = (uint8_t) (board_info.shuttle_id >> 8);
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + 7;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return rslt;
}

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 *
 */
static int16_t set_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length)
{
    enum coines_multi_io_pin pin_number;
    enum coines_pin_direction direction;
    enum coines_pin_value pin_value;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 3)
    {
        pin_number = (enum coines_multi_io_pin)payload[0];
        direction = (enum coines_pin_direction)payload[1];
        pin_value = (enum coines_pin_value)payload[2];
        rslt = coines_set_pin_config(pin_number, direction, pin_value);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return rslt;
}

/*!
 *  @brief This API is used to get the pin direction and pin state.
 *
 */
static int16_t get_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length)
{
    enum coines_multi_io_pin pin_number;
    enum coines_pin_direction direction;
    enum coines_pin_value pin_value;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 3)
    {
        pin_number = (enum coines_multi_io_pin)payload[0];
        rslt = coines_get_pin_config(pin_number, &direction, &pin_value);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + 3;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;
    resp[COINES_PROTO_PAYLOAD_POS] = pin_number;
    resp[COINES_PROTO_PAYLOAD_POS + 1] = direction;
    resp[COINES_PROTO_PAYLOAD_POS + 2] = pin_value;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 */
static int16_t set_vdd_vddio(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint16_t vdd = 0, vddio = 0;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 4)
    {
        memcpy(&vdd, payload, 2);
        memcpy(&vddio, &payload[2], 2);
        rslt = coines_set_shuttleboard_vdd_vddio_config(vdd, vddio);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the spi bus.
 *
 */
static int16_t spi_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length)
{
    enum coines_spi_bus bus;
    enum coines_spi_speed speed;
    enum coines_spi_mode mode;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 3)
    {
        bus = (enum coines_spi_bus)payload[0];
        speed = (enum coines_spi_speed)payload[1];
        mode = (enum coines_spi_mode)payload[2];
        rslt = coines_config_spi_bus(bus, speed, mode);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to de-configure the spi bus.
 *
 */
static int16_t spi_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 1)
    {
        bus = (enum coines_spi_bus)payload[0];
        rslt = coines_deconfig_spi_bus(bus);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the i2c bus.
 *
 */
static int16_t i2c_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length)
{
    enum coines_i2c_bus bus;
    enum coines_i2c_mode mode;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 2)
    {
        bus = (enum coines_i2c_bus)payload[0];
        mode = (enum coines_i2c_mode)payload[1];
        rslt = coines_config_i2c_bus(bus, mode);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to de-configure the i2c bus.
 *
 */
static int16_t i2c_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 1)
    {
        bus = (enum coines_i2c_bus)payload[0];
        rslt = coines_deconfig_i2c_bus(bus);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 *
 */
static int16_t i2c_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t dev_addr, reg_addr;
    uint16_t data_length;
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 6)
    {
        bus = (enum coines_i2c_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_write_i2c(bus, dev_addr, reg_addr, &payload[5], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 *
 */
static int16_t i2c_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    uint8_t dev_addr, reg_addr;
    uint16_t data_length;
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 5)
    {
        bus = (enum coines_i2c_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_read_i2c(bus, dev_addr, reg_addr, &resp[COINES_PROTO_PAYLOAD_POS], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + data_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 *
 */
static int16_t spi_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t cs_pin, reg_addr;
    uint16_t data_length;
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 6)
    {
        bus = (enum coines_spi_bus)payload[0];
        cs_pin = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_write_spi(bus, cs_pin, reg_addr, &payload[5], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 *
 */
static int16_t spi_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    uint8_t cs_pin, reg_addr;
    uint16_t data_length;
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 5)
    {
        bus = (enum coines_spi_bus)payload[0];
        cs_pin = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_read_spi(bus, cs_pin, reg_addr | 0x80, &resp[COINES_PROTO_PAYLOAD_POS], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + data_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

#if defined(MCU_APP30) || defined(MCU_APP31)
/*!
 *  @brief This API is used to write 16-bit register data on the I2C device.
 *
 */
static int16_t i2c_write_16bit_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t dev_addr;
    uint16_t reg_addr;
    uint16_t data_length;
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 6)
    {
        bus = (enum coines_i2c_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = ((uint16_t)payload[3] << 8) | payload[2]; //(msb << 8) | lsb;
        memcpy(&data_length, &payload[4], 2);
        rslt = coines_write_16bit_i2c(bus, dev_addr, reg_addr, &payload[6], data_length, COINES_I2C_TRANSFER_8BIT);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}


/*!
 *  @brief This API is used to read 16-bit register data from the I2C device.
 *
 */
static int16_t i2c_read_16bit_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    uint8_t dev_addr;
    uint16_t reg_addr;
    uint16_t data_length;
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 5)
    {
        bus = (enum coines_i2c_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = ((uint16_t)payload[3] << 8) | payload[2]; //(msb << 8) | lsb;
        memcpy(&data_length, &payload[4], 2);
        rslt = coines_read_16bit_i2c(bus, dev_addr, reg_addr, &resp[COINES_PROTO_PAYLOAD_POS], data_length,  COINES_I2C_TRANSFER_8BIT);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + data_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write 16-bit register data on the SPI device.
 *
 */
static int16_t spi_write_16bit_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t cs_pin;
    uint16_t reg_addr;
    uint16_t data_length;
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 6)
    {
        bus = (enum coines_spi_bus)payload[0];
        cs_pin = payload[1];
        reg_addr = ((uint16_t)payload[3] << 8) | payload[2]; //(msb << 8) | lsb;
        memcpy(&data_length, &payload[4], 2);
        rslt = coines_write_16bit_spi(bus, cs_pin, reg_addr, &payload[6], data_length, COINES_SPI_TRANSFER_8BIT);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read 16-bit register data from the SPI device.
 *
 */
static int16_t spi_read_16bit_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    uint8_t cs_pin;
    uint16_t reg_addr;
    uint16_t data_length;
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 5)
    {
        bus = (enum coines_spi_bus)payload[0];
        cs_pin = payload[1];
        reg_addr = ((uint16_t)payload[3] << 8) | payload[2]; //(msb << 8) | lsb;
        memcpy(&data_length, &payload[4], 2);
        rslt = coines_read_16bit_spi(bus, cs_pin, reg_addr, &resp[COINES_PROTO_PAYLOAD_POS], data_length, COINES_SPI_TRANSFER_8BIT);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + data_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}
#endif /* if defined(MCU_APP30) || defined(MCU_APP31) */

/*!
 *  @brief This API is used to trigger the softreset.
 *
 */
static int16_t soft_reset(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length)
{
    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 4)
    {
        soft_reset_triggered = true;
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;

}



static int16_t shuttle_eeprom_write(uint8_t cmd,
                                    uint8_t *payload,
                                    uint16_t payload_length,
                                    uint8_t *resp,
                                    uint16_t *resp_length)
{
    int16_t rslt;
    uint16_t start_addr;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    start_addr = payload[0];

    rslt = coines_shuttle_eeprom_write(start_addr, &payload[1], payload_length - 1);
    if (rslt != COINES_SUCCESS)
    {
        return rslt;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return rslt;

}

static int16_t shuttle_eeprom_read(uint8_t cmd,
                                   uint8_t *payload,
                                   uint16_t payload_length,
                                   uint8_t *resp,
                                   uint16_t *resp_length)
{
    int16_t rslt;
    uint16_t start_addr;
    uint8_t eeprom_data[EEPROM_RW_CONTENT_SIZE];

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    start_addr = payload[0];

    rslt = coines_shuttle_eeprom_read(start_addr, &eeprom_data[0], payload_length);
    if (rslt != COINES_SUCCESS)
    {
        return rslt;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + payload_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;
    memcpy(&resp[COINES_PROTO_PAYLOAD_POS], eeprom_data, payload_length);

    return COINES_SUCCESS;

}
