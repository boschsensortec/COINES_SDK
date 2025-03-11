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
 * @file streaming.c
 *
 * @brief This module provides APIs for handling streaming functionalities.
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
#include <pthread.h>
#include <time.h>

/*********************************************************************/
/* header files */
/*********************************************************************/
#include "streaming.h"
#include "mqueue.h"
#include "circular_buffer.h"

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
circular_buffer_t stream_cbuf;

/*********************************************************************/
/* static variables */
/*********************************************************************/
/*! Variable to hold sensor count */
static uint8_t coines_sensor_id_count = 0;

/*! Index for writing data into the payload buffer */
static uint8_t write_index = 0;

/*! Length of the response data */
static uint16_t resp_length;

/*! Length of the payload data */
static uint8_t payload_len = 0;

/*! Buffer to hold the payload data for streaming configuration */
static uint8_t payload[COINES_STREAM_CONFIG_BUFF_SIZE] = { 0 };

/*! Variable to hold streaming sensor info */
/* static struct coines_stream_sensor_info coines_sensor_info; */

/*! variable to hold the maximum no of streaming configuration buffer*/
static struct coines_streaming_settings coines_streaming_cfg_buf[COINES_MAX_SENSOR_ID];

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static bool streaming_init_success = false;

/*********************************************************************/
/* static functions */
/*********************************************************************/
static void common_stream_config(uint8_t sensor_id);
static int16_t interrupt_stream_config(uint8_t sensor_id);
static int16_t poll_stream_config(uint8_t sensor_id);
static int16_t dma_stream_config(uint8_t sensor_id);
static int16_t init_data_pipeline(uint8_t start_stop);
static int16_t deinit_data_pipeline(uint8_t start_stop);

/*********************************************************************/
/* functions */
/*********************************************************************/

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
    uint8_t sampling_unit[2];
    uint16_t gcd_sampling_time;
    enum coines_sampling_unit gcd_sampling_unit;
    uint32_t i;

    memset(payload, 0, sizeof(payload));

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

    payload[0] = coines_sensor_id_count;
    payload[1] = gcd_sampling_time >> 8;
    payload[2] = gcd_sampling_time & 0xFF;
    payload[3] = gcd_sampling_unit;

    ret = protocol_encode_packet(interface_type,
                                 COINES_CMD_ID_POLL_STREAM_COMMON,
                                 payload,
                                 COINES_POLL_STREAM_COMMON_PAYLOAD_LEN);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_POLL_STREAM_COMMON, resp_buffer, &resp_length);
        if (ret != COINES_SUCCESS)
        {
            return get_coines_error_mapping(ret);
        }
    }
    else
    {
        return get_coines_error_mapping(ret);
    }

    return COINES_SUCCESS;

}

/*!
 * @brief This API is used to initialize streaming settings based on stream mode
 *
 */
static int16_t stream_mode_init(enum coines_streaming_mode stream_mode, uint8_t* command)
{
    int16_t ret = COINES_SUCCESS;

    switch (stream_mode)
    {
        case COINES_STREAMING_MODE_DMA_INTERRUPT:
            *command = COINES_CMD_ID_DMA_INT_STREAM_CONFIG;
            break;

        case COINES_STREAMING_MODE_POLLING:
            *command = COINES_CMD_ID_POLL_STREAM_CONFIG;
            ret = config_poll_stream_timing();
            break;

        case COINES_STREAMING_MODE_INTERRUPT:
            *command = COINES_CMD_ID_INT_STREAM_CONFIG;
            break;
    }

    return ret;
}

/*!
 * @brief This API is used to configure streaming.
 *
 */
static int16_t configure_stream_mode(enum coines_streaming_mode stream_mode)
{
    int16_t ret;
    uint8_t command = 0;

    ret = stream_mode_init(stream_mode, &command);
    if (ret != COINES_SUCCESS)
    {
        return ret;
    }

    for (uint8_t sensor_id = 0; sensor_id < coines_sensor_id_count; sensor_id++)
    {
        /* Clear payload buffer */
        memset(&payload[0], 0x0, sizeof(payload));

        /* stream mode specific config */
        switch (stream_mode)
        {
            case COINES_STREAMING_MODE_DMA_INTERRUPT:
                (void)dma_stream_config(sensor_id);
                break;

            case COINES_STREAMING_MODE_POLLING:
                ret = poll_stream_config(sensor_id);
                break;

            case COINES_STREAMING_MODE_INTERRUPT:
                ret = interrupt_stream_config(sensor_id);
                break;
        }
        if (ret != COINES_SUCCESS)
        {
            return ret;
        }

        payload_len = write_index;

        /* Send stream config payload */
        ret = protocol_encode_packet(interface_type, command, &payload[0], payload_len);
        if (ret == COINES_SUCCESS)
        {
            ret = protocol_decode_packet(interface_type, command, resp_buffer, &resp_length);
            if (ret != COINES_SUCCESS)
            {
                return get_coines_error_mapping(ret);
            }
        }
        else
        {
            return get_coines_error_mapping(ret);
        }

        write_index = 0;
        payload_len = 0;
    }

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to configure common streaming settings
 */
static void common_stream_config(uint8_t sensor_id)
{
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].sensor_id;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.int_timestamp;
    payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[sensor_id].stream_config.intf;
    if (coines_streaming_cfg_buf[sensor_id].stream_config.intf == COINES_SENSOR_INTF_I2C)
    {
        payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[sensor_id].stream_config.i2c_bus;
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dev_addr;
    }
    else /* COINES_SENSOR_INTF_SPI */
    {
        payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[sensor_id].stream_config.spi_bus;
        payload[write_index++] = (uint8_t)coines_streaming_cfg_buf[sensor_id].stream_config.cs_pin;
    }
}

/*!
 * @brief This API is used to configure clear on write settings
 */
static void config_clear_on_write(uint8_t sensor_id)
{
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.clear_on_write_config.dummy_byte;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.clear_on_write_config.startaddress;
    payload[write_index++] =
        (uint8_t)coines_streaming_cfg_buf[sensor_id].stream_config.clear_on_write_config.num_bytes_to_clear;
}

/*!
 * @brief This API is used to configure interrupt line settings
 */
static void config_intline_info(uint8_t sensor_id)
{
    for (uint16_t j = 0; j < coines_streaming_cfg_buf[sensor_id].stream_config.intline_count; j++)
    {
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.intline_info[j];
    }
}

/*!
 * @brief This API is used to configure DMA streaming.
 *
 */
static int16_t dma_stream_config(uint8_t sensor_id)
{
    /* Common stream configuration */
    common_stream_config(sensor_id);

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.int_pin;

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.ctlr_addr >> 8;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.ctlr_addr & 0xFF;

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.startaddr_cmd >> 8;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.startaddr_cmd & 0xFF;

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.read_addr >> 8;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.read_addr & 0xFF;

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.read_len;

    /* no_of_bytes_read += coines_streaming_cfg_buf[sensor_id].stream_config.dma_config.read_len; */

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.spi_type;

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.hw_pin_state;

    /* no_of_bytes_read += 10; / * if extra bytes in resp packet, timestamp(6bytes) and packet no(4bytes) * / */

    /* coines_sensor_info.sensors_byte_count[sensor_id] = no_of_bytes_read; */
    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to configure polling streaming.
 *
 */
static int16_t poll_stream_config(uint8_t sensor_id)
{
    /* Common stream configuration */
    common_stream_config(sensor_id);

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.sampling_time >> 8;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.sampling_time & 0xFF;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.sampling_units;

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_blocks >> 8;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_blocks & 0xFF;

    for (uint16_t j = 0; j < coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_blocks; j++)
    {
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.reg_start_addr[j];
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_data_bytes[j] >> 8;
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_data_bytes[j] & 0xFF;
        if (coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_data_bytes[j] > MQUEUE_PACKET_SIZE)
        {
            return COINES_E_INVALID_PAYLOAD_LEN;
        }
    }

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.spi_type;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.clear_on_write;

    if (coines_streaming_cfg_buf[sensor_id].stream_config.clear_on_write)
    {
        config_clear_on_write(sensor_id);
    }

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.intline_count;

    if (coines_streaming_cfg_buf[sensor_id].stream_config.intline_count > 0)
    {
        config_intline_info(sensor_id);
    }
    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to configure interrupt streaming.
 *
 */
static int16_t interrupt_stream_config(uint8_t sensor_id)
{
    /* Common stream configuration */
    common_stream_config(sensor_id);

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.int_pin;

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_blocks >> 8;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_blocks & 0xFF;

    for (uint16_t j = 0; j < coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_blocks; j++)
    {
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.reg_start_addr[j];
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_data_bytes[j] >> 8;
        payload[write_index++] = coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_data_bytes[j] & 0xFF;
        if (coines_streaming_cfg_buf[sensor_id].data_blocks.no_of_data_bytes[j] > MQUEUE_PACKET_SIZE)
        {
            return COINES_E_INVALID_PAYLOAD_LEN;
        }
    }

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.spi_type;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.clear_on_write;
    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.hw_pin_state;

    if (coines_streaming_cfg_buf[sensor_id].stream_config.clear_on_write)
    {
        config_clear_on_write(sensor_id);
    }

    payload[write_index++] = coines_streaming_cfg_buf[sensor_id].stream_config.intline_count;

    if (coines_streaming_cfg_buf[sensor_id].stream_config.intline_count > 0)
    {
        config_intline_info(sensor_id);
    }
    return COINES_SUCCESS; 
}

/*!
 * @brief This API is used to initialize buffers and threads for streaming
 *
 */
static int16_t init_data_pipeline(uint8_t start_stop)
{
    int16_t ret;

    ret = circular_buffer_init(&stream_cbuf, CIRCULAR_BUFFER_SIZE);
    if (ret != COINES_SUCCESS)
    {
        return COINES_E_INIT_FAILED;
    }

    ret = mqueue_init();
    if (ret != COINES_SUCCESS)
    {
        return COINES_E_INIT_FAILED;
    }

    ret = protocol_decode_continuous_packets(interface_type, start_stop);
    if (ret != COINES_SUCCESS)
    {
        return get_coines_error_mapping(ret);
    }

    ret = protocol_decode_thread_start(interface_type);
    if (ret != COINES_SUCCESS)
    {
        return get_coines_error_mapping(ret);
    }

    return ret;
}

/*!
 * @brief This API is used to de-initialize buffers and threads for streaming
 *
 */
static int16_t deinit_data_pipeline(uint8_t start_stop)
{
    int16_t ret;

    ret = protocol_decode_continuous_packets(interface_type, start_stop);
    if (ret != COINES_SUCCESS)
    {
        return get_coines_error_mapping(ret);
    }

    ret = protocol_decode_thread_stop();
    if (ret != COINES_SUCCESS)
    {
        return get_coines_error_mapping(ret);
    }

    circular_buffer_free(&stream_cbuf);
    mqueue_deinit();
    coines_flush_intf(interface_type);

    return ret;
}

/*!
 * @brief This API is used to send streaming setting and start/stop the streaming.
 *
 */
int16_t coines_start_stop_streaming(enum coines_streaming_mode stream_mode, uint8_t start_stop)
{
    int16_t ret = COINES_SUCCESS;

    streaming_init_success = false;

    /*! Buffer to hold the formatted date-time string */
    char datetime_str[64];

    /*! Length of the formatted date-time string */
    uint16_t datetime_str_len;

    /*! Holds the current time as a time_t value */
    time_t system_time;

    /*! Pointer to a tm structure holding the decomposed time */
    struct tm *tm_local;

    /*check the if it is start request for polling streaming*/
    if (start_stop)
    {
        /* coines_sensor_info.no_of_sensors_enabled = coines_sensor_id_count; */

        ret = configure_stream_mode(stream_mode) ;
        if (ret != COINES_SUCCESS)
        {
            return ret;
        }

        ret = init_data_pipeline(start_stop);
        if (ret != COINES_SUCCESS)
        {
            return ret;
        }

        if (start_stop == COINES_USB_STREAMING_START)
        {
            payload[0] = 0xFF;
            ret = protocol_encode_packet(interface_type, COINES_CMD_ID_STREAM_START_STOP, &payload[0], 1);
        }
        else if (start_stop == COINES_EXT_FLASH_STREAMING_START)
        {
            /* Enable external flash logging */
            payload[0] = 0xFE;
            (void)time(&system_time);
            tm_local = localtime(&system_time);

            /* Format the current date and time as a string ("YYYY-MM-DD HH:MM:SS") */
            (void)strftime(datetime_str, sizeof(datetime_str), "StreamData_%Y-%m-%d_%H:%M:%S", tm_local);
            datetime_str_len = (uint16_t)strlen(datetime_str) + 1; /* 1 byte for null terminator */
            payload_len = (uint8_t)datetime_str_len + 1; /* 1 byte for the command */

            memcpy(&payload[1], datetime_str, datetime_str_len);
            ret = protocol_encode_packet(interface_type, COINES_CMD_ID_STREAM_START_STOP, &payload[0], payload_len);
        }

        if (ret == COINES_SUCCESS)
        {
            /* Ignoring the start stream response may sometimes fail at higher ODRs. If the start stream fails, 
            it will be captured as a READ_TIMEOUT error while reading the stream sensor data.*/
            // ret = protocol_decode_packet(interface_type, COINES_CMD_ID_STREAM_START_STOP, resp_buffer, &resp_length);
            ret = COINES_SUCCESS;
            streaming_init_success = true;
        }
    }
    else
    {
        ret = deinit_data_pipeline(start_stop);
        if (ret != COINES_SUCCESS)
        {
            return ret;
        }
        coines_delay_msec(100);
        memset(&payload[0], 0x0, sizeof(payload));
        ret = protocol_encode_packet(interface_type, COINES_CMD_ID_STREAM_START_STOP, &payload[0], 1);

        if (ret == COINES_SUCCESS)
        {
            do 
            {
                ret = protocol_decode_packet(interface_type, COINES_CMD_ID_STREAM_START_STOP, resp_buffer, &resp_length);
                if(ret == COINES_E_READ_TIMEOUT){
                    break;
                }
            }while(ret != COINES_SUCCESS);
            
            streaming_init_success = false;
        }
        else
        {
            return get_coines_error_mapping(ret);
        }


        /* clean-up */
        for (uint8_t i = 0; i < coines_sensor_id_count; i++)
        {
            memset(&coines_streaming_cfg_buf[i], 0, sizeof(struct coines_streaming_settings));
        }

        coines_sensor_id_count = 0;
    }

    return get_coines_error_mapping(ret);
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
    int16_t ret;
    uint32_t start_time;
    uint32_t current_time;
    struct coines_board_info board_info;
    uint32_t timeout = 0;
    
    start_time = coines_get_millis();

    if (data == NULL || valid_samples_count == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    if (number_of_samples == 0)
    {
        return COINES_E_INVALID_PARAM;
    }

    if (!streaming_init_success)
    {
        return COINES_E_STREAMING_INIT_FAILURE;
    }
    
    do
    {
        pthread_mutex_lock(&mutex);
        current_time = coines_get_millis();
        
        ret = mqueue_read_stream_data(sensor_id, data, number_of_samples, valid_samples_count);

        // Check cflag to determine the configured packet size. If it exceeds 255, assume fifo streaming.
        if (MQUEUE_PACKET_SIZE > 255)
        {
            // if cflag FIFO_STREAM_RSP_TIMEOUT is zero, use the default timeout defined in the library.
            if (FIFO_STREAM_RSP_TIMEOUT == 0)
            {
                timeout = FIFO_STREAM_RSP_TIMEOUT_MS;
            }
            else
            {
                timeout = FIFO_STREAM_RSP_TIMEOUT;
            }
        }
        else
        {
            // if cflag STREAM_RSP_TIMEOUT is zero, use the default timeout defined in the library.
            if (STREAM_RSP_TIMEOUT == 0)
            {
                timeout = STREAM_RSP_TIMEOUT_MS;
            }
            else
            {
                timeout = STREAM_RSP_TIMEOUT;
            }
        }

        /* Workaround to exit, when board is disconnected */
        if (((current_time - start_time) > timeout) && ret != COINES_SUCCESS)
        {
            /* Check whether the connection is alive, Send will return failure */
            ret = coines_get_board_info(&board_info);
            if (ret != COINES_SUCCESS)
            {
                (void)protocol_decode_continuous_packets(interface_type, COINES_STREAMING_STOP);
                (void)protocol_decode_thread_stop();
                coines_flush_intf(interface_type);
                pthread_mutex_unlock(&mutex);
                return COINES_E_DEVICE_NOT_FOUND;
            }
        }
        
        if (((current_time - start_time) > READ_TIMEOUT_MS) && ret != COINES_SUCCESS)
        {
            pthread_mutex_unlock(&mutex);
            return COINES_E_READ_TIMEOUT;
        }
        pthread_mutex_unlock(&mutex);
    } while (ret != MQUEUE_SUCCESS);

    return COINES_SUCCESS;

}

int16_t coines_trigger_timer(enum coines_timer_config tmr_cfg, enum coines_time_stamp_config ts_cfg)
{
    (void)tmr_cfg;
    (void)ts_cfg;

    return COINES_SUCCESS;
}
