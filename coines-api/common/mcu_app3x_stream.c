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
 * @file    mcu_app3x_stream.c
 * @date    Mar 10, 2023
 * @brief   COINES_SDK support file for mcu_app30.c
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "mcu_app3x_stream.h"
#include "mcu_app3x_interface.h"
#include "coines.h"
#include "mqueue.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
/*! Variable to hold sensor count */
static uint8_t coines_sensor_id_count = 0;

/*! Holds the interrupt line state */
static uint8_t stream_feature_line_state[32];

/*! variable to hold the maximum no of streaming configuration buffer*/
static struct coines_streaming_settings coines_stream_cfg[COINES_MAX_SENSOR_ID];

static uint32_t gst_period_us = 0;
static enum coines_streaming_mode stream_mode = COINES_STREAMING_MODE_POLLING;

static volatile uint32_t packet_counter[COINES_SHUTTLE_PIN_MAX] = { 0 };

static uint8_t streaming_resp_buff[1024];

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/
static void polling_timer_handler(void);
static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);
static void handler_feature_event(uint32_t multiio_pin, uint32_t multiio_pin_polarity);

static void read_int_stream_response(uint64_t timestamp, uint32_t multiio_pin, uint32_t packet_no);
static void read_poll_stream_response(void);

static int8_t (*sensor_read)(uint8_t, uint8_t, uint8_t, uint8_t*, uint16_t);
static int8_t (*sensor_write)(uint8_t, uint8_t, uint8_t, uint8_t*, uint16_t);

/*!
 * @brief This API is used to store the streaming settings
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
            stream_p = (struct coines_streaming_settings*)&coines_stream_cfg[coines_sensor_id_count];
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
int16_t config_poll_stream_sample_time(void)
{
    int16_t ret = COINES_SUCCESS;
    double sampling_time[2] = { 0, 0 };
    double remaining = 0;
    uint8_t sampling_unit[2];
    uint16_t gcd_sampling_time;
    enum coines_sampling_unit gcd_sampling_unit;
    struct coines_streaming_settings *stream_p;
    uint32_t i;

    /*check if sensor id count is greater than 1*/
    if (coines_sensor_id_count > 1)
    {
        for (i = 0; i < coines_sensor_id_count; i++)
        {
            sampling_time[i] = (double)coines_stream_cfg[i].stream_config.sampling_time;
            sampling_unit[i] = coines_stream_cfg[i].stream_config.sampling_units;
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
        gcd_sampling_time = coines_stream_cfg[coines_sensor_id_count - 1].stream_config.sampling_time;
        gcd_sampling_unit = coines_stream_cfg[coines_sensor_id_count - 1].stream_config.sampling_units;
    }

    if (gcd_sampling_unit == COINES_SAMPLING_TIME_IN_MICRO_SEC)
    {
        gst_period_us = gcd_sampling_time;
    }
    else if (gcd_sampling_unit == COINES_SAMPLING_TIME_IN_MILLI_SEC)
    {
        gst_period_us = gcd_sampling_time * 1000;
    }

    /* Update the multiplier */
    for (i = 0; i < coines_sensor_id_count; i++)
    {
        stream_p = &coines_stream_cfg[i];

        if (stream_p->stream_config.sampling_time < gst_period_us)
        {
            stream_p->gst_multiplier = 1;
        }
        else
        {
            stream_p->gst_multiplier = stream_p->stream_config.sampling_time / gst_period_us;
        }

        stream_p->gst_ticks_counter = stream_p->gst_multiplier;
    }

    return ret;
}

/*!
 * @brief This API is used to start the streaming.
 *
 */
static int16_t stream_start(void)
{
    int16_t rslt = COINES_SUCCESS;
    uint8_t poll_stream_count = 0;
    struct coines_streaming_settings *stream_p;
    enum coines_pin_interrupt_mode interrupt_mode;

    mqueue_init();
    if (stream_mode == COINES_STREAMING_MODE_POLLING)
    {
        config_poll_stream_sample_time();
    }

    for (uint8_t i = 0; i < coines_sensor_id_count; i++)
    {
        stream_p = &coines_stream_cfg[i];
        if (stream_mode == COINES_STREAMING_MODE_POLLING)
        {
            for (uint8_t int_line = 0; int_line < stream_p->stream_config.intline_count; int_line++)
            {
                if (stream_p->stream_config.intline_info[int_line] & 0x0F) /* Active high */
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
                }
                else/* Active low */
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
                }

                coines_attach_interrupt(
                    (enum coines_multi_io_pin)(stream_p->stream_config.intline_info[int_line] & 0x0F),
                    handler_feature_event,
                    interrupt_mode);
            }

            poll_stream_count++;
        }
        else if (stream_mode == COINES_STREAMING_MODE_INTERRUPT)
        {
            if (stream_p->stream_config.hw_pin_state)/* Active high */
            {
                interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
            }
            else/* Active low */
            {
                interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
            }

            rslt = coines_attach_timed_interrupt((enum coines_multi_io_pin)stream_p->stream_config.int_pin,
                                                 handler_drdy_int_event,
                                                 interrupt_mode);
        }
    }

    if (poll_stream_count > 0)
    {
        rslt = coines_timer_config(COINES_TIMER_INSTANCE_0, polling_timer_handler);
        if (rslt == COINES_SUCCESS)
        {
            rslt = coines_timer_start(COINES_TIMER_INSTANCE_0, gst_period_us);
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to stop the streaming.
 *
 */
static int16_t stream_stop(void)
{
    int16_t rslt = COINES_SUCCESS;
    uint8_t poll_stream_count = 0;
    struct coines_streaming_settings *stream_p;
    uint8_t interrupt_stream_count = 0;

    for (uint8_t i = 0; i < coines_sensor_id_count; i++)
    {
        stream_p = &coines_stream_cfg[i];

        if (stream_mode == COINES_STREAMING_MODE_POLLING)
        {
            for (uint8_t int_line = 0; int_line < stream_p->stream_config.intline_count; int_line++)
            {
                coines_detach_interrupt((enum coines_multi_io_pin)(stream_p->stream_config.intline_info[int_line]));
            }

            poll_stream_count++;
        }
        else if (stream_mode == COINES_STREAMING_MODE_INTERRUPT)
        {
            interrupt_stream_count++;
            coines_detach_timed_interrupt((enum coines_multi_io_pin)stream_p->stream_config.int_pin);
        }
        else /* COINES_STREAM_MODE_FIFO_POLLING */
        {
            /* TODO */
        }

        /* Clear the streaming configuration */
        memset(&coines_stream_cfg[i], 0, sizeof(struct coines_streaming_settings));
    }

    if (poll_stream_count > 0)
    {
        rslt = coines_timer_stop(COINES_TIMER_INSTANCE_0);
    }

    mqueue_deinit();
    coines_sensor_id_count = 0;

    return rslt;
}

/*!
 * @brief This API is used to send the streaming settings to the board.
 */
int16_t coines_start_stop_streaming(enum coines_streaming_mode mode, uint8_t start_stop)
{
    int16_t ret = COINES_SUCCESS;

    stream_mode = mode;

    if (start_stop == COINES_USB_STREAMING_START)
    {
        stream_start();
    }
    else if (start_stop == COINES_STREAMING_STOP)
    {
        stream_stop();
    }

    return ret;
}

/*!
 * @brief This API will be called on polling timer expire.
 *
 */
static void polling_timer_handler(void)
{
    read_poll_stream_response();
}

/*!
 *  @brief This API will be called on data ready interrupt
 *
 */
static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin_polarity;
    packet_counter[multiio_pin] = packet_counter[multiio_pin] + 1;
    read_int_stream_response(timestamp, multiio_pin, packet_counter[multiio_pin]);

}

/*!
 * @brief This API is used to configure interface type I2C/SPI.
 *
 */
static void config_sensor_intf(enum coines_sensor_intf intf_type)
{
    if (intf_type == COINES_SENSOR_INTF_I2C)
    {
        sensor_read = &coines_read_i2c;
        sensor_write = &coines_write_i2c;
    }
    else
    {
        sensor_read = &coines_read_spi;
        sensor_write = &coines_write_spi;
    }
}

/*!
 * @brief This function handles feature event
 */
static void handler_feature_event(uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin;
    (void)multiio_pin_polarity;
}

/*!
 * @brief This API is used to send the polling streaming response
 *
 */
static void read_poll_stream_response(void)
{
    struct coines_streaming_settings *stream_p;
    int8_t ret = COINES_SUCCESS;
    int8_t rslt = COINES_SUCCESS;
    uint8_t read_mask = 0;
    uint8_t intf_bus;
    uint8_t intf_addr;
    uint8_t sensor_data_pos;
    uint8_t feature_int_info_pos;
    uint8_t len;

    if (coines_sensor_id_count > 0)
    {
        for (uint8_t i = 0; i < coines_sensor_id_count; i++)
        {
            stream_p = &coines_stream_cfg[i];
            if (stream_p->gst_ticks_counter > 0)
            {
                stream_p->gst_ticks_counter--;
            }

            if (stream_p->gst_ticks_counter == 0)
            {
                sensor_data_pos = 0;
                
                /* reload tick-counter */
                stream_p->gst_ticks_counter = stream_p->gst_multiplier;

                config_sensor_intf(stream_p->stream_config.intf);
                if (stream_p->stream_config.intf == COINES_SENSOR_INTF_SPI)
                {
                    intf_bus = stream_p->stream_config.spi_bus;
                    intf_addr = stream_p->stream_config.cs_pin;
                    read_mask = 0x80;
                }
                else
                {
                    intf_bus = stream_p->stream_config.i2c_bus;
                    intf_addr = stream_p->stream_config.dev_addr;
                }

                if (stream_p->stream_config.clear_on_write)
                {
                    /* Dummy byte information also read based on the input */
                    ret = sensor_read(intf_bus,
                                      intf_addr,
                                      stream_p->stream_config.clear_on_write_config.startaddress | read_mask,
                                      stream_p->stream_config.clear_on_write_config.data_buf,
                                      (stream_p->stream_config.clear_on_write_config.num_bytes_to_clear +
                                       stream_p->stream_config.clear_on_write_config.dummy_byte));
                }

                for (uint8_t index_t = 0; index_t < stream_p->data_blocks.no_of_blocks; index_t++)
                {
                    rslt = sensor_read(intf_bus,
                                       intf_addr,
                                       stream_p->data_blocks.reg_start_addr[index_t] | read_mask,
                                       &streaming_resp_buff[sensor_data_pos],
                                       stream_p->data_blocks.no_of_data_bytes[index_t]);

                    sensor_data_pos += stream_p->data_blocks.no_of_data_bytes[index_t];

                }

                if ((stream_p->stream_config.clear_on_write) && (COINES_SUCCESS == ret))
                {
                    /* Ignoring dummy byte if present and writing to register for clearing status register */
                    ret = sensor_write(intf_bus,
                                       intf_addr,
                                       stream_p->stream_config.clear_on_write_config.startaddress,
                                       &stream_p->stream_config.clear_on_write_config.data_buf[stream_p->stream_config.
                                                                                               clear_on_write_config.
                                                                                               dummy_byte],
                                       stream_p->stream_config.clear_on_write_config.num_bytes_to_clear);
                }

                feature_int_info_pos = sensor_data_pos;

                /* Get the Interrupt line information and update them in the buffer for transmission */
                if (stream_p->stream_config.intline_count != 0)
                {
                    for (uint8_t int_line = 0; int_line < stream_p->stream_config.intline_count; int_line++)
                    {
                        if (stream_feature_line_state[stream_p->stream_config.intline_info[int_line]] == 1)
                        {
                            stream_p->DATA_intline[int_line] = 1;
                            stream_feature_line_state[stream_p->stream_config.intline_info[int_line]] = 0;
                        }
                        else
                        {
                            stream_p->DATA_intline[int_line] = 0;
                        }
                    }

                    memcpy(&streaming_resp_buff[feature_int_info_pos],
                           stream_p->DATA_intline,
                           stream_p->stream_config.intline_count);
                    feature_int_info_pos += stream_p->stream_config.intline_count;

                }

                if (rslt == COINES_SUCCESS)
                {
                    len = feature_int_info_pos;
                    mqueue_add_data(stream_p->sensor_id, streaming_resp_buff, len);
                }
            }
        }
    }
}

/*!
 * @brief This API is used to send the interrupt streaming response
 *
 */
static void read_int_stream_response(uint64_t timestamp, uint32_t multiio_pin, uint32_t packet_no)
{
    uint8_t sensor_data_pos;
    uint8_t feature_int_info_pos;
    uint8_t packet_no_pos = 0;
    uint8_t timestamp_pos = 0;
    struct coines_streaming_settings *stream_p;
    int8_t ret = COINES_SUCCESS;
    int8_t rslt = COINES_SUCCESS;
    uint8_t intf_bus;
    uint8_t intf_addr;
    uint8_t read_mask = 0;
    uint64_t timestamp_us = timestamp / 1000;
    uint8_t len;

    if (coines_sensor_id_count > 0)
    {
        for (uint8_t i = 0; i < coines_sensor_id_count; i++)
        {
            stream_p = &coines_stream_cfg[i];
            if (stream_p->stream_config.int_pin == multiio_pin)
            {
                sensor_data_pos = 0;

                /* Packet number 4bytes */
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((packet_no & 0xff000000) >> 24);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((packet_no & 0x00ff0000) >> 16);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((packet_no & 0x0000ff00) >> 8);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) (packet_no & 0x000000ff);
                sensor_data_pos = packet_no_pos;

                config_sensor_intf(stream_p->stream_config.intf);
                if (stream_p->stream_config.intf == COINES_SENSOR_INTF_SPI)
                {
                    intf_bus = stream_p->stream_config.spi_bus;
                    intf_addr = stream_p->stream_config.cs_pin;
                    read_mask = 0x80;
                }
                else
                {
                    intf_bus = stream_p->stream_config.i2c_bus;
                    intf_addr = stream_p->stream_config.dev_addr;
                }

                if (stream_p->stream_config.clear_on_write)
                {
                    /* Dummy byte information also read based on the input */
                    ret = sensor_read(intf_bus,
                                      intf_addr,
                                      stream_p->stream_config.clear_on_write_config.startaddress | read_mask,
                                      stream_p->stream_config.clear_on_write_config.data_buf,
                                      (stream_p->stream_config.clear_on_write_config.num_bytes_to_clear +
                                       stream_p->stream_config.clear_on_write_config.dummy_byte));
                }

                for (uint8_t index_t = 0; index_t < stream_p->data_blocks.no_of_blocks; index_t++)
                {

                    rslt = sensor_read(intf_bus,
                                       intf_addr,
                                       stream_p->data_blocks.reg_start_addr[index_t] | read_mask,
                                       &streaming_resp_buff[sensor_data_pos],
                                       stream_p->data_blocks.no_of_data_bytes[index_t]);

                    sensor_data_pos += stream_p->data_blocks.no_of_data_bytes[index_t];
                }

                if ((stream_p->stream_config.clear_on_write) && (COINES_SUCCESS == ret))
                {
                    /* Ignoring dummy byte if present and writing to register for clearing status register */
                    ret = sensor_write(intf_bus,
                                       intf_addr,
                                       stream_p->stream_config.clear_on_write_config.startaddress,
                                       &stream_p->stream_config.clear_on_write_config.data_buf[stream_p->stream_config.
                                                                                               clear_on_write_config.
                                                                                               dummy_byte],
                                       stream_p->stream_config.clear_on_write_config.num_bytes_to_clear);
                }

                timestamp_pos = sensor_data_pos;
                if (stream_p->stream_config.int_timestamp)
                {
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 40);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 32);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 24);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 16);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 8);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us);
                }

                feature_int_info_pos = timestamp_pos;

                /* Get the Interrupt line information and update them in the buffer for transmission */
                if (stream_p->stream_config.intline_count != 0)
                {
                    for (uint8_t int_line = 0; int_line < stream_p->stream_config.intline_count; int_line++)
                    {
                        if (stream_feature_line_state[stream_p->stream_config.intline_info[int_line]] == 1)
                        {
                            stream_p->DATA_intline[int_line] = 1;
                            stream_feature_line_state[stream_p->stream_config.intline_info[int_line]] = 0;
                        }
                        else
                        {
                            stream_p->DATA_intline[int_line] = 0;
                        }
                    }

                    memcpy(&streaming_resp_buff[feature_int_info_pos],
                           stream_p->DATA_intline,
                           stream_p->stream_config.intline_count);
                    feature_int_info_pos += stream_p->stream_config.intline_count;

                }

                if (rslt == COINES_SUCCESS)
                {
                    len = feature_int_info_pos;
                    mqueue_add_data(stream_p->sensor_id, streaming_resp_buff, len);
                }
            }
        }
    }
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

    mqueue_read_stream_data(sensor_id, data, valid_samples_count);

    return COINES_SUCCESS;
}
