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
 * @file    comm_intf.c
 * @brief This module provides communication interface layer between host and application board using
 * various interfaces (USB,VCOM,BLE)
 *
 */

/*!
 * @defgroup comm_intf_api comm_intf
 * @{*/

/*********************************************************************/
/* system header files */
/*********************************************************************/
#ifdef PLATFORM_LINUX
/* To enable usleep in glibc */
#define _DEFAULT_SOURCE
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef PLATFORM_LINUX
#include <unistd.h>
#endif

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines_defs.h"
#include "comm_intf.h"
#include "comm_ringbuffer.h"
#include "usb.h"
#include "mutex_port.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! global communication buffer*/
coines_command_t comm_buf;

/*! @brief MUTEX between the communication data buffer and processing. */
mutex_t comm_intf_thread_mutex;

/*! @brief MUTEX the non streaming data buffer and processing. */
mutex_t comm_intf_non_stream_buff_mutex;

/*! @brief MUTEX for the streaming data buffer and processing.*/
mutex_t comm_intf_stream_buff_mutex;

/*! variable to hold the streaming info */
comm_stream_info_t comm_intf_sensor_info;

/*********************************************************************/
/* static variables */
/*********************************************************************/

/*! Interface state for each interface type */
static uint8_t is_interface_usb_init = 0;

/*
 * TODO:static uint8_t is_interface_vcom_init = 0;
 * TODO:static uint8_t is_interface_ble_init = 0;
 */

static comm_ringbuffer_t* rb_stream_rsp_p[COINES_MAX_SENSOR_COUNT];
static comm_ringbuffer_t* rb_gpio_rsp_p;
static comm_ringbuffer_t* rb_non_stream_rsp_p;

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/

#define COMM_INTF_RETRY_COUNT            1000

#define COMM_INTF_STREAMING_RETRY_COUNT  10

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

static void comm_intf_data_receive_call_back(usb_rsp_buffer_t* rsp_buf);
static void comm_intf_parse_received_data(usb_rsp_buffer_t *rsp);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 * @brief This API is used to open communication interface
 */
int16_t comm_intf_open(enum coines_comm_intf intf_type, coines_board_t* board_type)
{
    uint32_t idx;
    int16_t rslt = COINES_SUCCESS;

    if (board_type == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    switch (intf_type)
    {
        case COINES_COMM_INTF_USB:
            if (is_interface_usb_init)
            {
                /* USB interface already initialized - de-init and then re-init */
                comm_intf_close(intf_type);
            }

            /* allocate ringbuffers */
            for (idx = 0; idx < COINES_MAX_SENSOR_COUNT; idx++)
            {
                rb_stream_rsp_p[idx] = comm_ringbuffer_create(COMM_INTF_RSP_BUF_SIZE);
                if (!rb_stream_rsp_p[idx])
                {
                    return COINES_E_MEMORY_ALLOCATION;
                }
            }

            rb_non_stream_rsp_p = comm_ringbuffer_create(COMM_INTF_RSP_BUF_SIZE);
            if (!rb_non_stream_rsp_p)
            {
                return COINES_E_MEMORY_ALLOCATION;
            }

            rb_gpio_rsp_p = comm_ringbuffer_create(COMM_INTF_RSP_BUF_SIZE);
            if (!rb_gpio_rsp_p)
            {
                return COINES_E_MEMORY_ALLOCATION;
            }

            /* init pthread objects */
            mutex_init(&comm_intf_thread_mutex);
            mutex_init(&comm_intf_non_stream_buff_mutex);
            mutex_init(&comm_intf_stream_buff_mutex);

            /* init usb device */
            rslt = usb_open_device(&comm_buf, comm_intf_data_receive_call_back);
            if (rslt != COINES_SUCCESS)
            {
                return rslt;
            }

            *board_type = comm_buf.board_type;

            is_interface_usb_init = 1;

            break;

        case COINES_COMM_INTF_VCOM:
            rslt = COINES_E_NOT_SUPPORTED;
            break;

        case COINES_COMM_INTF_BLE:
            rslt = COINES_E_NOT_SUPPORTED;
            break;
        default:
            break;
    }

    return rslt;
}

/*!
 * @brief This API is used to close communication interface
 */
void comm_intf_close(enum coines_comm_intf intf_type)
{
    uint32_t idx;

    switch (intf_type)
    {
        case COINES_COMM_INTF_USB:
            usb_close_device();
            mutex_destroy(&comm_intf_non_stream_buff_mutex);
            mutex_destroy(&comm_intf_stream_buff_mutex);
            mutex_destroy(&comm_intf_thread_mutex);

            for (idx = 0; idx < COINES_MAX_SENSOR_COUNT; idx++)
            {
                comm_ringbuffer_delete(rb_stream_rsp_p[idx]);
                rb_stream_rsp_p[idx] = NULL;
            }

            comm_ringbuffer_delete(rb_non_stream_rsp_p);
            rb_non_stream_rsp_p = NULL;
            comm_ringbuffer_delete(rb_gpio_rsp_p);
            rb_gpio_rsp_p = NULL;

            is_interface_usb_init = 0;
            break;

        case COINES_COMM_INTF_VCOM:
            break;

        case COINES_COMM_INTF_BLE:
            break;
    }
}

/*!
 * @brief This API is used as a data receive callback
 *
 * @param[in] rsp_buf: pointer to response buffer
 *
 * @return void
 */
static void comm_intf_data_receive_call_back(usb_rsp_buffer_t* rsp_buf)
{
    mutex_lock(&comm_intf_thread_mutex);
    comm_intf_parse_received_data(rsp_buf);
    mutex_unlock(&comm_intf_thread_mutex);
}

/*!
 * @brief This API is used to Initialize the command header
 */
void comm_intf_init_command_header(uint8_t cmd_type, uint8_t int_feature)
{
    comm_buf.buffer[0] = COINES_CMD_ID;
    comm_buf.buffer[1] = 0;
    comm_buf.buffer[2] = cmd_type;
    comm_buf.buffer[3] = int_feature;
    comm_buf.buffer_size = 4;
}

/*!
 * @brief This API is used to write the uint8_t data into command buffer
 */
void comm_intf_put_u8(uint8_t data)
{
    comm_buf.buffer[comm_buf.buffer_size++] = data;
}

/*!
 * @brief This API is used to write the uint16_t data into command buffer
 */
void comm_intf_put_u16(uint16_t data)
{
    comm_buf.buffer[comm_buf.buffer_size++] = data >> 8;
    comm_buf.buffer[comm_buf.buffer_size++] = data & 0xFF;
}

/*!
 * @brief This API is used to write the uint32_t data into command buffer
 */
void comm_intf_put_u32(uint32_t data)
{
    comm_buf.buffer[comm_buf.buffer_size++] = (data >> 24) & 0xFF;
    comm_buf.buffer[comm_buf.buffer_size++] = (data >> 16) & 0xFF;
    comm_buf.buffer[comm_buf.buffer_size++] = (data >> 8) & 0xFF;
    comm_buf.buffer[comm_buf.buffer_size++] = data & 0xFF;
}

/*!
 * @brief This API is used to send and get the command response from board
 *        When 'rsp_buf' parameter is NULL,the API doesn't sends the command but doesn't
 *        read-out the response.
 */
int16_t comm_intf_send_command(coines_rsp_buffer_t* rsp_buf)
{
    int16_t rslt;

    /*if board type is development desktop add line termination characters*/
    comm_intf_put_u8('\r');
    comm_intf_put_u8('\n');
    comm_buf.buffer[1] = (uint8_t)comm_buf.buffer_size;
    rslt = usb_send_command(&comm_buf);

    if (rsp_buf == NULL)
    {
        return rslt;
    }

    if (rslt == COINES_SUCCESS)
    {
        rslt = comm_intf_process_non_streaming_response(rsp_buf);
    }

    return rslt;
}

/*!
 * @brief This API is used to trigger/stop the streaming feature
 */
int16_t comm_intf_start_stop_streaming(uint8_t state, comm_stream_info_t *sensor_info)
{
    uint32_t idx;
    int16_t rslt = COINES_SUCCESS;

    if (sensor_info == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    if (state)
    {
        comm_intf_sensor_info.no_of_sensors_enabled = sensor_info->no_of_sensors_enabled;
        for (idx = 0; idx < COINES_MAX_SENSOR_COUNT; idx++)
        {
            comm_intf_sensor_info.sensors_byte_count[idx] = sensor_info->sensors_byte_count[idx];
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to process non streaming response
 *
 * @param[in] rsp_buf: pointer to response buffer
 *
 * @return Result of API execution status
 */
int16_t comm_intf_process_non_streaming_response(coines_rsp_buffer_t * rsp_buf)
{
    int16_t rslt = COINES_SUCCESS;
    uint32_t retry_count = COMM_INTF_RETRY_COUNT;

    mutex_lock(&comm_intf_non_stream_buff_mutex);

    while (retry_count > 0)
    {
        retry_count--;
        if (rb_non_stream_rsp_p->packetCounter > 0)
        {
            rsp_buf->buffer_size = comm_ringbuffer_read(rb_non_stream_rsp_p, rsp_buf->buffer, 1);
            if (rsp_buf->buffer[0] == COINES_DD_RESP_ID)
            {
                if ((rsp_buf->buffer_size > 0) && (rsp_buf->buffer_size != COINES_INVALID_DATA))
                {
                    memset(rsp_buf->buffer + rsp_buf->buffer_size, 0, COINES_DATA_BUF_SIZE - rsp_buf->buffer_size);
                }

                break;
            }
        }
        else
        {
            comm_intf_delay(1);
        }
    }

    if (rsp_buf->buffer_size == 0)
    {
        rslt = COINES_E_FAILURE;
    }

    mutex_unlock(&comm_intf_non_stream_buff_mutex);

    return rslt;
}

/*!
 * @brief This API is used to process the streaming response
 */
int16_t comm_intf_process_stream_response(uint8_t sensor_id, uint32_t no_ofsamples, coines_stream_rsp_buffer_t* rsp_buf)
{
    (void)no_ofsamples;

    int16_t rslt = COINES_SUCCESS;
    uint32_t retry_count = COMM_INTF_STREAMING_RETRY_COUNT;

    if (rsp_buf == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    if ((sensor_id > COINES_MAX_SENSOR_ID) || (sensor_id < COINES_MIN_SENSOR_ID))
    {
        return COINES_E_NOT_SUPPORTED;
    }

    mutex_lock(&comm_intf_stream_buff_mutex);

    /* wait for data */
    while (retry_count > 0)
    {
        /* if data is present in buffer, exit the loop, else wait another cycle */
        if (rb_stream_rsp_p[sensor_id - 1]->packetCounter > 0)
        {
            break;
        }

        comm_intf_delay(1);
        retry_count--;
    }

    /* if any data came before wait period expired, then process it, else return error */
    if (rb_stream_rsp_p[sensor_id - 1]->packetCounter > 0)
    {
        mutex_lock(&comm_intf_thread_mutex);

        /*
         * TODO: do we really need packet counter ? It seems nobody cares about how many samples are requested. In the
         * end all of them are returned,
         * and also the separator gets completely removed, so it's just a continuous data buffer
         */

        rsp_buf->buffer_size =
            comm_ringbuffer_read(rb_stream_rsp_p[sensor_id - 1],
                                 rsp_buf->buffer,
                                 rb_stream_rsp_p[sensor_id - 1]->packetCounter);
        mutex_unlock(&comm_intf_thread_mutex);

        if (rsp_buf->buffer_size > 0)
        {
            /*TODO: do we really need to zero-out the memory ? If yes, careful with the size to avoid a 0 size */
            memset(rsp_buf->buffer + rsp_buf->buffer_size, 0, COINES_STREAM_RSP_BUF_SIZE - rsp_buf->buffer_size);
        }
        else
        {
            rslt = COINES_E_FAILURE;
        }
    }
    else
    {
        rslt = COINES_E_FAILURE;
    }

    mutex_unlock(&comm_intf_stream_buff_mutex);

    return rslt;
}

/*!
 * @brief This API is used to parse the received data
 *
 * @param[in] buffer: pointer to buffer
 *
 * @return void
 */
static void comm_intf_parse_received_data(usb_rsp_buffer_t*rsp)
{
    int16_t rslt = COINES_SUCCESS;
    uint32_t pkt_len;
    uint16_t sensor_identifier, sid_mask;
    uint16_t lsb, msb, data_pos, bytes_to_w;
    uint16_t index_t = 0;
    uint8_t *buffer;
    uint8_t stream_type, id_mask_shift;

    if (rsp == NULL)
    {
        return;
    }

    buffer = rsp->buffer;

    while (index_t < COINES_DATA_BUF_SIZE)
    {
        if (buffer[COINES_DD_COMMAND_ID_RESPONSE_POSITION] != COINES_EXTENDED_READ_RESPONSE_ID)
        {
            pkt_len = buffer[index_t + COINES_BYTEPOS_PACKET_SIZE];
        }
        else
        {
            /* 13 -> header 11 bytes and packet delimiter 2 bytes */
            pkt_len = COINES_CALC_PACKET_LENGTH(buffer[COINES_BYTEPOS_LEN_MSB], buffer[COINES_BYTEPOS_LEN_LSB]) + 13;
        }

        /* basic packet integrity checks (header ID, packet length, terminator*/
        if ((buffer[index_t] == COINES_DD_RESP_ID) && (pkt_len > 0) && (buffer[index_t + pkt_len - 1] == '\n'))
        {
            DEBUG_PRINT("index: %d - ", index_t);
            DEBUG_PRINT_BUF(buffer + index_t, pkt_len);
            DEBUG_PRINT("\n");

            stream_type = buffer[index_t + 4];

            /* Check if packet is a stream data packet or normal packet */
            if (stream_type == COINES_RSPID_POLLING_STREAMING_DATA)
            {
                /* Stream data packet - polling */

                /* Check if the packet contains a successful response */
                if (buffer[index_t + COINES_RESPONSE_STATUS_POSITION] == COINES_SUCCESS)
                {
                    lsb = buffer[index_t + pkt_len - 3];
                    msb = buffer[index_t + pkt_len - 4];
                    sid_mask = (uint16_t)((msb << 8) | lsb);
                    data_pos = index_t + 5;

                    /* cycle through all IDs in the sid_mask */
                    for (id_mask_shift = 0; id_mask_shift < comm_intf_sensor_info.no_of_sensors_enabled;
                         id_mask_shift++)
                    {
                        sensor_identifier = (uint16_t)(1 << id_mask_shift);
                        if (sid_mask & sensor_identifier)
                        {
                            /* if ID is in the mask, and if it's a valid ID, then handle it */
                            if ((sensor_identifier >= COINES_MIN_SENSOR_ID) &&
                                (sensor_identifier <= COINES_MAX_SENSOR_ID))
                            {
                                DEBUG_PRINT("data byte position: %d sensor id: %d \n", data_pos, sensor_identifier);
                                bytes_to_w = comm_intf_sensor_info.sensors_byte_count[sensor_identifier - 1];
                                rslt =
                                    comm_ringbuffer_write_packet(rb_stream_rsp_p[sensor_identifier - 1],
                                                                 &buffer[data_pos],
                                                                 bytes_to_w);
                                data_pos += bytes_to_w;
                            }
                        }
                    }
                }
            }
            else if (stream_type == COINES_RSPID_INT_STREAMING_DATA)
            {
                /* Stream data packet - interrupt */

                /* Check if the packet contains a successful response */
                if (buffer[index_t + COINES_RESPONSE_STATUS_POSITION] == COINES_SUCCESS)
                {
                    /* interrupt streaming type */
                    sensor_identifier = buffer[index_t + 5];
                    data_pos = index_t + 6;

                    /* handle only known sensor IDs */
                    if ((sensor_identifier >= COINES_MIN_SENSOR_ID) && (sensor_identifier <= COINES_MAX_SENSOR_ID))
                    {
                        DEBUG_PRINT("data byte position: %d sensor id: %d \n", data_pos, sensor_identifier);
                        bytes_to_w = comm_intf_sensor_info.sensors_byte_count[sensor_identifier - 1];
                        rslt = comm_ringbuffer_write_packet(rb_stream_rsp_p[sensor_identifier - 1],
                                                            &buffer[data_pos],
                                                            bytes_to_w);
                    }
                }
            }
            else
            {
                /* Non stream data packet */

                rslt = comm_ringbuffer_write_packet(rb_non_stream_rsp_p, &buffer[index_t], pkt_len);
            }

            /* If there is any error in ring buffer writing, exit parsing the packet*/
            if (rslt != COINES_SUCCESS)
            {
                break;
            }
        }

        index_t += 64;
    }
}

/*!
 * @brief This API is used for introducing a delay in milliseconds
 */
void comm_intf_delay(uint32_t delay_ms)
{
#ifdef PLATFORM_LINUX
    uint32_t delay_microsec = (uint32_t)(delay_ms * 1000);
    usleep(delay_microsec);
#endif

#ifdef PLATFORM_WINDOWS
    Sleep(delay_ms);
#endif
}
