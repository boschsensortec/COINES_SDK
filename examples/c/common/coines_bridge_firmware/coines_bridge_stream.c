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
#include "coines_bridge_stream.h"
#include "coines_bridge_client.h"
#include "stream.h"
#include "job_queue.h"
#include "mbuf.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#ifdef DEBUG
#define USB_DEBUG_PIN COINES_MINI_SHUTTLE_PIN_2_7
#define USB_DEBUG_PIN_2 COINES_MINI_SHUTTLE_PIN_2_5
#endif

#define POLL_INT_STREAM_READ_LEN 255

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
extern enum coines_comm_intf comm_intf;

/* Workaround - bhi3 fifo streaming as larger fifo len around 10k for first interrupt
so cannot be handled with mbuf, transmit data for every read(framelength) and skip executing job queue in coines_yield */
extern bool bhi_streaming_configured;

extern uint64_t transfer_timeout;

extern bool ext_flash_log_enabled;

FILE *stream_data_file;

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
static struct coines_stream_settings stream_config =
{ .gst_period_us = 0, .ts_mode = COINES_STREAM_NO_TIMESTAMP, .stream_mode = COINES_STREAM_MODE_POLLING };

/*! Holds streaming config */
static union coines_streaming streaming[STREAM_MAX_COUNT_T];

/*! Buffer used to hold the streaming response data */
uint8_t streaming_resp_buff[STREAM_BUFF_SIZE];

/*! Hold streaming sensor count */
static uint8_t stream_count = 0;

/*! Updated on polling timer0 expire */
static volatile uint8_t poll_stream_triggered = 0;

/*! Holds the interrupt line state */
static uint8_t stream_feature_line_state[COINES_SHUTTLE_PIN_MAX];

static volatile uint32_t packet_counter[COINES_SHUTTLE_PIN_MAX] = { 0 };

static bool mbuf_read_pending = false;

static bool int_fifo_streaming =false;

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/
static int16_t streaming_start(void);
static int16_t streaming_stop(void);
void polling_timer_handler(void);
static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);
static void handler_feature_event(uint32_t multiio_pin, uint32_t multiio_pin_polarity);
static void send_int_stream_response(uint8_t *p_data);
static void send_poll_stream_response(void);
static void send_dma_int_stream_response(uint8_t *p_data);
static bool is_stream_sensor_configured(uint8_t sensor_id);
void mbuf_user_evt_handler(mbuf_evt_type_t event);
static void send_intterupt_fifo_response(uint8_t *buffer, uint16_t length); 

int8_t (*sensor_read)(uint8_t, uint8_t, uint8_t, uint8_t*, uint16_t);
int8_t (*sensor_write)(uint8_t, uint8_t, uint8_t, uint8_t*, uint16_t);

/*!
 * @brief This API is used to get and configure polling stream settings.
 *
 */
int16_t  poll_streaming_config(uint8_t cmd,
                               uint8_t *payload,
                               uint16_t payload_length,
                               uint8_t *resp,
                               uint16_t *resp_length)
{
    uint8_t read_index = 0;
    uint16_t total_block_read_len = 0;
    uint16_t raw_time;
    union coines_streaming *stream_p = &streaming[stream_count];

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 12)
    {
        if (!is_stream_sensor_configured(payload[0]))
        {
            if (stream_count < STREAM_MAX_COUNT_T)
            {
                stream_p->poll_config.sensor_id = payload[read_index++];
                stream_p->poll_config.timestamp = payload[read_index++];
                stream_p->poll_config.intf = (enum coines_sensor_intf)payload[read_index++];
                stream_p->poll_config.intf_bus = (uint8_t)payload[read_index++];
                stream_p->poll_config.intf_addr = (uint8_t)payload[read_index++];
                stream_p->poll_config.sampling_time = (uint16_t)payload[read_index++] << 8; /* sampling_time msb */
                stream_p->poll_config.sampling_time |= (uint16_t)payload[read_index++] & 0xFF; /* sampling_time lsb
                                                                                                  */
                stream_p->poll_config.sampling_units = (enum coines_sampling_unit)payload[read_index++];
                stream_p->poll_config.no_of_blocks = (uint16_t)payload[read_index++] << 8;
                stream_p->poll_config.no_of_blocks |= (uint16_t)payload[read_index++] & 0xFF;
                if (stream_p->poll_config.no_of_blocks > 0)
                {
                    for (uint16_t i = 0 ; i < stream_p->poll_config.no_of_blocks; i++ )
                    {
                        stream_p->poll_config.reg_start_addr[i] = payload[read_index++];
                        stream_p->poll_config.no_of_data_bytes[i] = payload[read_index++] << 8;
                        stream_p->poll_config.no_of_data_bytes[i] |= payload[read_index++] & 0xFF;
                        total_block_read_len += stream_p->int_config.no_of_data_bytes[i];
                    }
                }
                else
                {
                    return COINES_E_STREAM_INVALID_BLOCK_SIZE;
                }

                if (total_block_read_len >= COM_STREAM_BUFF_SIZE)
                {
                    return COINES_E_STREAM_INVALID_BLOCK_SIZE;
                } 

                stream_p->poll_config.spi_type = payload[read_index++];
                stream_p->poll_config.clear_on_write = payload[read_index++];
                if (stream_p->poll_config.clear_on_write)
                {
                    stream_p->poll_config.clear_on_write_config.dummy_byte = payload[read_index++];
                    stream_p->poll_config.clear_on_write_config.startaddress = payload[read_index++];
                    stream_p->poll_config.clear_on_write_config.num_bytes_to_clear = payload[read_index++];
                }

                stream_p->poll_config.intline_count = payload[read_index++];
                if (stream_p->poll_config.intline_count > 0)
                {
                    for (uint8_t i = 0 ; i < stream_p->poll_config.intline_count; i++ )
                    {
                        stream_p->poll_config.intline_info[i] = payload[read_index++];
                    }
                }

                raw_time = stream_p->poll_config.sampling_time;
                if (stream_p->poll_config.sampling_units == (uint8_t)COINES_SAMPLING_TIME_IN_MICRO_SEC)
                {
                    stream_p->poll_config.sampling_period_us = raw_time;
                }
                else
                {
                    stream_p->poll_config.sampling_period_us = raw_time * 1000;
                }

                if (stream_p->poll_config.sampling_period_us <= stream_config.gst_period_us)
                {
                    stream_p->poll_config.gst_multiplier = 1;
                }
                else
                {
                    stream_p->poll_config.gst_multiplier = stream_p->poll_config.sampling_period_us /
                                                           stream_config.gst_period_us;
                }

                stream_p->poll_config.gst_ticks_counter = stream_p->poll_config.gst_multiplier;

                stream_count++;
            }
            else
            {
                return COINES_E_STREAM_CONFIG_MEMORY_FULL;
            }
        }
        else
        {
            return COINES_E_STREAM_SENSOR_ALREADY_CONFIGURED;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    stream_config.stream_mode = COINES_STREAM_MODE_POLLING;
    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to configure global sampling timer period for all polling streaming.
 *
 */
int16_t poll_streaming_common(uint8_t cmd,
                              uint8_t *payload,
                              uint16_t payload_length,
                              uint8_t *resp,
                              uint16_t *resp_length)
{
    uint16_t raw_time;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 4)
    {
        raw_time = (payload[1] << 8) | payload[2];
        if (raw_time > 0)
        {
            if (payload[3] == (uint8_t)COINES_SAMPLING_TIME_IN_MICRO_SEC)
            {
                stream_config.gst_period_us = raw_time;
            }
            else if (payload[3] == (uint8_t)COINES_SAMPLING_TIME_IN_MILLI_SEC)
            {
                stream_config.gst_period_us = raw_time * 1000;
            }
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
 * @brief This API is used to start/stop the streaming.
 *
 */
int16_t streaming_start_stop(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    int16_t rslt = COINES_SUCCESS;
    #if defined(MCU_APP30) || defined(MCU_APP31)
    char date_time_str[100];
    #endif

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 1)
    {
        if (stream_count > 0)
        {
            if (payload[0] == 0x00)
            {
                rslt = streaming_stop();
                #if defined(MCU_APP30) || defined(MCU_APP31)
                if (ext_flash_log_enabled)
                {
                    (void)fflush(stdout);
                    fclose(stream_data_file);
                    ext_flash_log_enabled = false;
                }
                #endif
            }
            else if (payload[0] == 0xFF)
            {
                rslt = streaming_start();
            }
            #if defined(MCU_APP30) || defined(MCU_APP31)
            else if (payload[0]== 0xFE)
            {
                ext_flash_log_enabled = true;

                // Adjust for command byte at the start
                payload_length -= 1; 

                if (payload_length > 2) 
                {
                    // Adjust for command byte and null terminator, copy into date_time_str
                    memcpy(date_time_str, &payload[1], payload_length - 2); 
                    date_time_str[payload_length - 2] = '\0'; 
                    strcat(date_time_str, ".txt");
                    stream_data_file = fopen(date_time_str, "wb"); 
                }
                rslt = streaming_start();
            }
            #endif
        }
        else
        {
            return COINES_E_STREAM_NOT_CONFIGURED;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    if (rslt != COINES_SUCCESS)
    {
        return rslt;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + payload_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;
    memcpy(&resp[COINES_PROTO_PAYLOAD_POS], payload, payload_length);

    return rslt;
}

/*!
 * @brief This API will be called on polling timer expire.
 *
 */
void polling_timer_handler(void)
{
    ++poll_stream_triggered;
}

/*!
 * @brief This API is used to start the streaming.
 *
 */
static int16_t streaming_start(void)
{
    int16_t rslt = COINES_SUCCESS;
    uint8_t poll_stream_count = 0;
    union coines_streaming *stream_p;
    enum coines_pin_interrupt_mode interrupt_mode;

    if ((stream_count > 0) && (stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT || stream_config.stream_mode == COINES_STREAM_MODE_DMA_INTERRUPT))
    {
        /* Intialize job_queue and mbuf for interrupt streaming */
        (void)job_queue_init();
        (void)mbuf_init(mbuf_user_evt_handler);
    }

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_p = &streaming[i];
        if (stream_config.stream_mode == COINES_STREAM_MODE_POLLING)
        {
            for (uint8_t int_line = 0; int_line < stream_p->poll_config.intline_count; int_line++)
            {
                if (stream_p->poll_config.intline_info[int_line] & COINES_STREAM_INT_PIN_STATE_MASK) /* Active high */
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
                }
                else/* Active low */
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
                }

                coines_attach_interrupt((enum coines_multi_io_pin)(stream_p->poll_config.intline_info[int_line] &
                                                                   COINES_STREAM_INT_PIN_MASK),
                                        handler_feature_event,
                                        interrupt_mode);
            }

            poll_stream_count++;
        }
        else if ((stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT) || (stream_config.stream_mode == COINES_STREAM_MODE_DMA_INTERRUPT))
        {
            if (stream_p->int_config.hw_pin_state)/* Active high */
            {
                interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
            }
            else/* Active low */
            {
                interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
            }

            rslt = coines_attach_timed_interrupt((enum coines_multi_io_pin)stream_p->int_config.int_pin,
                                                 handler_drdy_int_event,
                                                 interrupt_mode);
        }
    }

    if (poll_stream_count > 0)
    {
        rslt = coines_timer_config(COINES_TIMER_INSTANCE_0, polling_timer_handler);
        if (rslt == COINES_SUCCESS)
        {
            rslt = coines_timer_start(COINES_TIMER_INSTANCE_0, stream_config.gst_period_us);
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to stop the streaming.
 *
 */
static int16_t streaming_stop(void)
{
    int16_t rslt = COINES_SUCCESS;
    uint8_t poll_stream_count = 0;
    union coines_streaming *stream_p;
    uint8_t interrupt_stream_count = 0;

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_p = &streaming[i];

        if (stream_config.stream_mode == COINES_STREAM_MODE_POLLING)
        {
            for (uint8_t int_line = 0; int_line < stream_p->poll_config.intline_count; int_line++)
            {
                coines_detach_interrupt((enum coines_multi_io_pin)(stream_p->poll_config.intline_info[int_line]));
            }

            poll_stream_count++;
        }
        else if ((stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT) ||(stream_config.stream_mode == COINES_STREAM_MODE_DMA_INTERRUPT))
        {
            interrupt_stream_count++;
            coines_detach_timed_interrupt((enum coines_multi_io_pin)stream_p->int_config.int_pin);
        }
        else /* COINES_STREAM_MODE_FIFO_POLLING */
        {
            /* TODO */
        }

        /* Clear the streaming configuration */
        memset(&streaming[i], 0, sizeof(union coines_streaming));
    }

    if (poll_stream_count > 0)
    {
        rslt = coines_timer_stop(COINES_TIMER_INSTANCE_0);
    }
    else if (interrupt_stream_count > 0)
    {
        /* Deinit the job_queue and mbuf */
        job_queue_deinit();
        mbuf_deinit();
        memset((void *)packet_counter, 0, sizeof(packet_counter));
    }

    stream_count = 0;

    return rslt;
}

/*!
 * @brief This API is used to configure interface type I2C/SPI.
 *
 */
void config_sensor_intf(enum coines_sensor_intf intf_type)
{
    /*lint -e64*/
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

    /*lint +e64*/
}

/*!
 * @brief This API is used to send the polling streaming response
 *
 */
static void send_poll_stream_response(void)
{
    uint16_t buffer_idx = 0;
    uint16_t resp_length;
    union coines_streaming *stream_p;
    int8_t ret = COINES_SUCCESS;
    int8_t rslt = COINES_SUCCESS;
    uint8_t read_mask = 0;

    if (stream_count > 0)
    {
        --poll_stream_triggered;
        for (uint8_t i = 0; i < stream_count; i++)
        {
            stream_p = &streaming[i];
            if (stream_p->poll_config.gst_ticks_counter > 0)
            {
                stream_p->poll_config.gst_ticks_counter--;
            }

            if (stream_p->poll_config.gst_ticks_counter == 0)
            {
                /* reload tick-counter */
                stream_p->poll_config.gst_ticks_counter = stream_p->poll_config.gst_multiplier;

                /* frame polling streaming response */
                streaming_resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
                streaming_resp_buff[COINES_PROTO_CMD_POS] = COINES_READ_SENSOR_DATA;
                streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = stream_p->poll_config.sensor_id;
                buffer_idx = COINES_PROTO_PAYLOAD_POS + 1;

                config_sensor_intf(stream_p->poll_config.intf);

                if (stream_p->poll_config.intf == COINES_SENSOR_INTF_SPI)
                {
                    read_mask = 0x80;
                }

                if (stream_p->poll_config.clear_on_write)
                {
                    /* Dummy byte information also read based on the input */
                    ret = sensor_read((enum coines_i2c_bus)stream_p->poll_config.intf_bus,
                                      stream_p->poll_config.intf_addr,
                                      stream_p->poll_config.clear_on_write_config.startaddress | read_mask,
                                      stream_p->poll_config.clear_on_write_config.data_buf,
                                      (stream_p->poll_config.clear_on_write_config.num_bytes_to_clear +
                                       stream_p->poll_config.clear_on_write_config.dummy_byte));
                }

                for (uint8_t index_t = 0; index_t < stream_p->poll_config.no_of_blocks; index_t++)
                {
                    rslt = sensor_read((enum coines_i2c_bus)stream_p->poll_config.intf_bus,
                                       stream_p->poll_config.intf_addr,
                                       stream_p->poll_config.reg_start_addr[index_t] | read_mask,
                                       &streaming_resp_buff[buffer_idx],
                                       stream_p->poll_config.no_of_data_bytes[index_t]);

                    buffer_idx += stream_p->poll_config.no_of_data_bytes[index_t];
                }

                if ((stream_p->poll_config.clear_on_write) && (COINES_SUCCESS == ret))
                {
                    /* Ignoring dummy byte if present and writing to register for clearing status register */
                    ret = sensor_write((enum coines_i2c_bus)stream_p->poll_config.intf_bus,
                                       stream_p->poll_config.intf_addr,
                                       stream_p->poll_config.clear_on_write_config.startaddress,
                                       &stream_p->poll_config.clear_on_write_config.data_buf[stream_p->poll_config.
                                                                                             clear_on_write_config.
                                                                                             dummy_byte],
                                       stream_p->poll_config.clear_on_write_config.num_bytes_to_clear);
                }

                /* Get the Interrupt line information and update them in the buffer for transmission */
                if (stream_p->int_config.intline_count != 0)
                {
                    for (uint8_t int_line = 0; int_line < stream_p->poll_config.intline_count; int_line++)
                    {
                        if (stream_feature_line_state[stream_p->poll_config.intline_info[int_line]] == 1)
                        {
                            stream_p->poll_config.DATA_intline[int_line] = 1;
                            stream_feature_line_state[stream_p->poll_config.intline_info[int_line]] = 0;
                        }
                        else
                        {
                            stream_p->poll_config.DATA_intline[int_line] = 0;
                        }
                    }

                    memcpy(&streaming_resp_buff[buffer_idx],
                           stream_p->poll_config.DATA_intline,
                           stream_p->poll_config.intline_count);
                           buffer_idx += stream_p->poll_config.intline_count;
                }

                if (rslt == COINES_SUCCESS)
                {
                    resp_length = buffer_idx;
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    coines_write_intf(comm_intf, streaming_resp_buff, resp_length);
                }
                else
                {
                    /* streaming block read failed */
                    resp_length = 4;
                    streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = 0xFF; /* set invalid sensor id */
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    coines_write_intf(comm_intf, streaming_resp_buff, resp_length);
                }
            }
        }
    }
}

/*!
 * @brief This API is used to check streaming is configured and triggered
 *
 */
void send_streaming_response(void)
{
    if ((stream_config.stream_mode == COINES_STREAM_MODE_POLLING) && poll_stream_triggered)
    {
        send_poll_stream_response();
    }
    else if ((stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT) || (stream_config.stream_mode == COINES_STREAM_MODE_DMA_INTERRUPT))
    {
        (void)job_queue_execute_jobs();
    }
    else /* COINES_STREAM_MODE_FIFO_POLLING */
    {
        /* TODO - Implements*/
    }
}

/*!
 *  @brief This API will be called on data ready interrupt
 *
 */
static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin_polarity;
    coines_bridge_job_data_t job_data;

    if(stream_count > 0)
    {
        packet_counter[multiio_pin] = packet_counter[multiio_pin] + 1;
        job_data.multiio_pin = multiio_pin;
        job_data.packet_no = packet_counter[multiio_pin];
        job_data.timestamp_us = timestamp / 1000;

        if (stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT)
        {
            if (job_queue_add_job(send_int_stream_response, (uint8_t *)&job_data, true) != JOB_QUEUE_SUCCESS)
            {
                /* TODO - Need to handle if job queue is full */
            }
        }
        else if (stream_config.stream_mode == COINES_STREAM_MODE_DMA_INTERRUPT)
        {
            if (job_queue_add_job(send_dma_int_stream_response, (uint8_t *)&job_data, true) != JOB_QUEUE_SUCCESS)
            {
                /* TODO - Need to handle if job queue is full */
            }
        }
    } 
}

/*!
 * @brief This function handles feature event
 */
static void handler_feature_event(uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin_polarity;

    if (multiio_pin < COINES_SHUTTLE_PIN_MAX)
    {
        stream_feature_line_state[multiio_pin] = 1;

        /* TODO: use polarity */
    }
}

/*!
 * @brief This API is used to send the interrupt streaming response
 *
 */
static void send_intterupt_fifo_response(uint8_t *buffer, uint16_t length) 
{
    if (int_fifo_streaming) 
    {
        coines_write_intf(comm_intf, buffer, length);
    } else {
        mbuf_add_to_buffer(buffer, length);
    }
}

/*!
 * @brief This API is used to send the interrupt streaming response
 *
 */
static void send_int_stream_response(uint8_t *p_data)
{
    uint16_t buffer_idx = 0;
    uint16_t resp_length;
    union coines_streaming *stream_p;
    int8_t ret = COINES_SUCCESS;
    int8_t rslt = COINES_SUCCESS;
    uint8_t read_mask = 0;
    /*lint -e{826}*/
    coines_bridge_job_data_t *job_data = (coines_bridge_job_data_t *)p_data;

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_p = &streaming[i];
        if (stream_p->int_config.int_pin == job_data->multiio_pin)
        {
            
            if (job_data->packet_no == packet_counter[job_data->multiio_pin])
            {
				#ifdef DEBUG
                coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_6, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
                #endif
                streaming_resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
                streaming_resp_buff[COINES_PROTO_CMD_POS] = COINES_READ_SENSOR_DATA;
                streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = stream_p->int_config.sensor_id;
                buffer_idx = COINES_PROTO_PAYLOAD_POS + 1;

                /* Packet number 4bytes */
                streaming_resp_buff[buffer_idx++] = (uint8_t) ((job_data->packet_no & 0xff000000) >> 24);
                streaming_resp_buff[buffer_idx++] = (uint8_t) ((job_data->packet_no & 0x00ff0000) >> 16);
                streaming_resp_buff[buffer_idx++] = (uint8_t) ((job_data->packet_no & 0x0000ff00) >> 8);
                streaming_resp_buff[buffer_idx++] = (uint8_t) (job_data->packet_no & 0x000000ff);

                config_sensor_intf(stream_p->int_config.intf);

                if (stream_p->int_config.intf == COINES_SENSOR_INTF_SPI)
                {
                    read_mask = 0x80;
                }

                if (stream_p->int_config.clear_on_write)
                {
                    /* Dummy byte information also read based on the input */
                    ret = sensor_read((uint8_t)stream_p->int_config.intf_bus,
                                      stream_p->int_config.intf_addr,
                                      stream_p->int_config.clear_on_write_config.startaddress | read_mask,
                                      stream_p->int_config.clear_on_write_config.data_buf,
                                      (stream_p->int_config.clear_on_write_config.num_bytes_to_clear +
                                       stream_p->int_config.clear_on_write_config.dummy_byte));
                }

                for (uint8_t index_t = 0; index_t < stream_p->int_config.no_of_blocks; index_t++)
                {

                    rslt = sensor_read((uint8_t)stream_p->int_config.intf_bus,
                                       stream_p->int_config.intf_addr,
                                       stream_p->int_config.reg_start_addr[index_t] | read_mask,
                                       &streaming_resp_buff[buffer_idx],
                                       stream_p->int_config.no_of_data_bytes[index_t]);

                    buffer_idx += stream_p->int_config.no_of_data_bytes[index_t];
                }

                if ((stream_p->int_config.clear_on_write) && (COINES_SUCCESS == ret))
                {
                    /* Ignoring dummy byte if present and writing to register for clearing status register */
                    ret = sensor_write((uint8_t)stream_p->int_config.intf_bus,
                                       stream_p->int_config.intf_addr,
                                       stream_p->int_config.clear_on_write_config.startaddress,
                                       &stream_p->int_config.clear_on_write_config.data_buf[stream_p->int_config.
                                                                                            clear_on_write_config.
                                                                                            dummy_byte],
                                       stream_p->int_config.clear_on_write_config.num_bytes_to_clear);
                }

                if (stream_p->int_config.timestamp)
                {
                    streaming_resp_buff[buffer_idx++] = (uint8_t) (job_data->timestamp_us >> 40);
                    streaming_resp_buff[buffer_idx++] = (uint8_t) (job_data->timestamp_us >> 32);
                    streaming_resp_buff[buffer_idx++] = (uint8_t) (job_data->timestamp_us >> 24);
                    streaming_resp_buff[buffer_idx++] = (uint8_t) (job_data->timestamp_us >> 16);
                    streaming_resp_buff[buffer_idx++] = (uint8_t) (job_data->timestamp_us >> 8);
                    streaming_resp_buff[buffer_idx++] = (uint8_t) (job_data->timestamp_us);
                }

                /* Get the Interrupt line information and update them in the buffer for transmission */
                if (stream_p->int_config.intline_count != 0)
                {
                    for (uint8_t int_line = 0; int_line < stream_p->int_config.intline_count; int_line++)
                    {
                        if (stream_feature_line_state[stream_p->int_config.intline_info[int_line]] == 1)
                        {
                            stream_p->int_config.DATA_intline[int_line] = 1;
                            stream_feature_line_state[stream_p->int_config.intline_info[int_line]] = 0;
                        }
                        else
                        {
                            stream_p->int_config.DATA_intline[int_line] = 0;
                        }
                    }

                    memcpy(&streaming_resp_buff[buffer_idx],
                           stream_p->int_config.DATA_intline,
                           stream_p->int_config.intline_count);
                    buffer_idx += stream_p->int_config.intline_count;

                }

                if (rslt == COINES_SUCCESS)
                {
                    resp_length = buffer_idx;
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    send_intterupt_fifo_response(streaming_resp_buff, resp_length);
                }
                else
                {
                    /* streaming block read failed */
                    resp_length = 4;
                    streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = 0xFF; /* set invalid sensor id */
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    send_intterupt_fifo_response(streaming_resp_buff, resp_length);
                }
				#ifdef DEBUG
                coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_6, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
                #endif
            }
        }
    }
}

/*!
 * @brief This API is used to transmit the buffered streaming data
 *
 */
void transmit_streaming_rsp(void)
{
    uint64_t curr_timeout;
    uint8_t *active_buff = NULL;
    uint16_t active_buff_len = 0;

    curr_timeout = coines_get_millis();

    if (mbuf_get_buffer_length() > 0 && (curr_timeout - transfer_timeout) >= STREAM_TRANSFER_TIMEOUT)
    {
        transfer_timeout = curr_timeout;
        mbuf_read_pending = true;
    }

    if (mbuf_read_pending)
    {
        (void)mbuf_get_from_buffer(&active_buff, &active_buff_len);
        coines_execute_critical_region(switch_buffer);
        if (ext_flash_log_enabled)
        {
            fwrite(active_buff, 1, active_buff_len, stream_data_file);
        }
        else
        {
            #ifdef DEBUG
            coines_set_pin_config(USB_DEBUG_PIN, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
            #endif
            coines_write_intf(comm_intf, active_buff, active_buff_len);
            #ifdef DEBUG
            coines_set_pin_config(USB_DEBUG_PIN, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
            #endif
        }
        mbuf_update_processed_buffer();
        mbuf_read_pending = false;
        
    }
}

/*!
 * @brief This API is used to execute the task in cpu idle time
 *
 */
void coines_yield(void)
{
    if ((stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT) || (stream_config.stream_mode == COINES_STREAM_MODE_DMA_INTERRUPT) ||/* New protocol */
        (stream_settings.stream_mode == STREAM_MODE_INTERRUPT)) /* Legacy protocol */
    {
        if (bhi_streaming_configured || int_fifo_streaming)
        {
            return;
        }
        
        (void)job_queue_execute_jobs();
    }
}

/*!
 * @brief This API is used to get and configure interrupt stream settings
 *
 */
int16_t int_streaming_config(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t read_index = 0;
    uint16_t total_block_read_len = 0;
    uint16_t i;
    union coines_streaming *stream_p = &streaming[stream_count];

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 12)
    {
        if (stream_count < STREAM_MAX_COUNT_T)
        {
            stream_p->int_config.sensor_id = payload[read_index++];
            stream_p->int_config.timestamp = payload[read_index++];
            stream_p->int_config.intf = (enum coines_sensor_intf)payload[read_index++];
            stream_p->int_config.intf_bus = payload[read_index++];
            stream_p->int_config.intf_addr = payload[read_index++];
            stream_p->int_config.int_pin = payload[read_index++];

            stream_p->int_config.no_of_blocks = payload[read_index++] << 8; /* no_of_blocks byte msb */
            stream_p->int_config.no_of_blocks |= payload[read_index++] & 0xFF; /* no_of_blocks byte lsb */
            if (stream_p->int_config.no_of_blocks > 0)
            {
                for (i = 0 ; i < stream_p->int_config.no_of_blocks; i++ )
                {
                    stream_p->int_config.reg_start_addr[i] = payload[read_index++];
                    stream_p->int_config.no_of_data_bytes[i] = payload[read_index++] << 8;
                    stream_p->int_config.no_of_data_bytes[i] |= payload[read_index++] & 0xFF;
                    total_block_read_len += stream_p->int_config.no_of_data_bytes[i];
                }
            }
            else
            {
                return COINES_E_STREAM_INVALID_BLOCK_SIZE;
            }
            if (total_block_read_len >= COM_STREAM_BUFF_SIZE)
            {
                return COINES_E_STREAM_INVALID_BLOCK_SIZE;
            } 
            if (total_block_read_len >= POLL_INT_STREAM_READ_LEN)
            {
                int_fifo_streaming = true;
            }

            stream_p->int_config.spi_type = payload[read_index++];
            stream_p->int_config.clear_on_write = payload[read_index++];
            stream_p->int_config.hw_pin_state = payload[read_index++] & 0x01;
            if (stream_p->int_config.clear_on_write)
            {
                stream_p->int_config.clear_on_write_config.dummy_byte = payload[read_index++];
                stream_p->int_config.clear_on_write_config.startaddress = payload[read_index++];
                stream_p->int_config.clear_on_write_config.num_bytes_to_clear = payload[read_index++];
            }

            stream_p->int_config.intline_count = payload[read_index++];
            if (stream_p->int_config.intline_count > 0)
            {
                for (i = 0 ; i < stream_p->int_config.intline_count; i++ )
                {
                    stream_p->int_config.intline_info[i] = payload[read_index++];
                }
            }

            stream_config.stream_mode = COINES_STREAM_MODE_INTERRUPT;
            stream_count++;

        }
        else
        {
            return COINES_E_STREAM_CONFIG_MEMORY_FULL;
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
 * @brief This API is used to check whether streming sensor is already configured or not
 *
 */
static bool is_stream_sensor_configured(uint8_t sensor_id)
{
    uint8_t *stream_sensor_id;

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_sensor_id = (uint8_t *)&streaming[i];
        if (*stream_sensor_id == sensor_id)
        {
            return true;
        }
    }

    return false;
}

void mbuf_user_evt_handler(mbuf_evt_type_t event)
{
    if (event == MBUF_EVT_BUFFER_FULL)
    {
        #ifdef DEBUG
        coines_set_pin_config(USB_DEBUG_PIN_2, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
        #endif
        mbuf_read_pending = true;
        #ifdef DEBUG
        coines_set_pin_config(USB_DEBUG_PIN_2, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
        #endif
    }
}

/*!
 * @brief This API is used to get and configure dma interrupt stream settings
 *
 */
int16_t dma_int_streaming_config(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t read_index = 0;
    union coines_streaming *stream_p = &streaming[stream_count];

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 15)
    {
        if (stream_count < STREAM_MAX_COUNT_T)
        {
            stream_p->int_config.sensor_id = payload[read_index++];
            stream_p->int_config.timestamp = payload[read_index++];
            stream_p->int_config.intf = (enum coines_sensor_intf)payload[read_index++];
            stream_p->int_config.intf_bus = payload[read_index++];
            stream_p->int_config.intf_addr = payload[read_index++];
            stream_p->int_config.int_pin = payload[read_index++];

            stream_p->int_config.dma.ctlr_addr = payload[read_index++] << 8;
            stream_p->int_config.dma.ctlr_addr |= payload[read_index++] & 0xFF;

            stream_p->int_config.dma.startaddr_cmd = payload[read_index++] << 8;
            stream_p->int_config.dma.startaddr_cmd |= payload[read_index++] & 0xFF;

            stream_p->int_config.dma.read_addr = payload[read_index++] << 8;
            stream_p->int_config.dma.read_addr |= payload[read_index++] & 0xFF;

            stream_p->int_config.dma.read_len = payload[read_index++];
           
            stream_p->int_config.spi_type = payload[read_index++];
            
            stream_p->int_config.hw_pin_state = payload[read_index++] & 0x01;
           
            stream_config.stream_mode = COINES_STREAM_MODE_DMA_INTERRUPT;
            stream_count++;

        }
        else
        {
            return COINES_E_STREAM_CONFIG_MEMORY_FULL;
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
 * @brief This API is used to send the interrupt streaming response
 *
 */
static void send_dma_int_stream_response(uint8_t *p_data)
{
    uint8_t dma_data_pos = 0;
    uint16_t resp_length;
    uint8_t packet_no_pos = 0;
    uint8_t timestamp_pos = 0;
    union coines_streaming *stream_p;
    int8_t ret = COINES_SUCCESS;
    int8_t rslt = COINES_SUCCESS;
    uint8_t read_mask = 0;
    uint8_t data[2];
    /*lint -e{826}*/
    coines_bridge_job_data_t *job_data = (coines_bridge_job_data_t *)p_data;

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_p = &streaming[i];
        if (stream_p->int_config.int_pin == job_data->multiio_pin)
        {

            if (job_data->packet_no == packet_counter[job_data->multiio_pin])
            {
                streaming_resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
                streaming_resp_buff[COINES_PROTO_CMD_POS] = COINES_READ_SENSOR_DATA;
                streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = stream_p->int_config.sensor_id;
                packet_no_pos = COINES_PROTO_PAYLOAD_POS + 1;

                /* Packet number 4bytes */
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((job_data->packet_no & 0xff000000) >> 24);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((job_data->packet_no & 0x00ff0000) >> 16);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((job_data->packet_no & 0x0000ff00) >> 8);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) (job_data->packet_no & 0x000000ff);
                dma_data_pos = packet_no_pos;

                config_sensor_intf(stream_p->int_config.intf);

                if (stream_p->int_config.intf == COINES_SENSOR_INTF_SPI)
                {
                    read_mask = 0x80;
                }


                /* Setup DMA controller */
                data[0] = stream_p->int_config.dma.startaddr_cmd & 0x00FF;
                data[1] = stream_p->int_config.dma.startaddr_cmd >> 8;
                ret = sensor_write((uint8_t)stream_p->int_config.intf_bus,
                                    stream_p->int_config.intf_addr,
                                    (uint8_t)stream_p->int_config.dma.ctlr_addr, 
                                    data,
                                    2);
                
                /* Read DMA data */
                if (ret == COINES_SUCCESS)
                {
                        ret = sensor_read((uint8_t)stream_p->int_config.intf_bus,
                                      stream_p->int_config.intf_addr,
                                      (uint8_t)stream_p->int_config.dma.read_addr | read_mask,
                                      &streaming_resp_buff[dma_data_pos],
                                      stream_p->int_config.dma.read_len);
                        dma_data_pos += stream_p->int_config.dma.read_len;
                    
                }

                /* Terminate the DMA */
                if (ret == COINES_SUCCESS)
                { 
                    data[1] = 0;
                    ret = sensor_write((uint8_t)stream_p->int_config.intf_bus,
                                       stream_p->int_config.intf_addr,
                                       (uint8_t)stream_p->int_config.dma.ctlr_addr,
                                       data,
                                       2);
                }               

                timestamp_pos = dma_data_pos;
                if (stream_p->int_config.timestamp)
                {
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (job_data->timestamp_us >> 40);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (job_data->timestamp_us >> 32);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (job_data->timestamp_us >> 24);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (job_data->timestamp_us >> 16);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (job_data->timestamp_us >> 8);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (job_data->timestamp_us);
                }

                if (rslt == COINES_SUCCESS)
                {
                    resp_length = timestamp_pos;
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    mbuf_add_to_buffer(streaming_resp_buff, resp_length);
                }
                else
                {
                    /* streaming block read failed */
                    resp_length = 4;
                    streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = 0xFF; /* set invalid sensor id */
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    mbuf_add_to_buffer(streaming_resp_buff, resp_length);
                }
            }
        }
    }
}
