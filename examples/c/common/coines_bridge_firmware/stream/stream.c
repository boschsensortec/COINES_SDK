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
 * @file        stream.c
 *
 * @brief       Sensor data streaming over usb.
 *
 */
/*!
 * @ingroup APPLICATION
 *
 * @defgroup STREAM stream
 * @{
 * @details

 *
 **/

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**********************************************************************************/
/* own header files */
/**********************************************************************************/
#include "coines.h"
#include "stream.h"
#include "job_queue.h"
#include "mbuf.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
/*!  To store the GCD time, mode and the type of streaming */
stream_settings_t stream_settings =
{ .GST_period_us = 0, .ts_mode = STREAM_NO_TIMESTAMP, .stream_mode = STREAM_MODE_POLLING };

/*! Gives how many active sensor streaming in progress */
uint32_t stream_active_count = 0;

/*! Holds steam parameters for each sensor */
stream_descriptor_t stream_descriptors[STREAM_MAX_COUNT];

/*! Holds FIFO stream parameters */
stream_fifo_descriptor_t stream_fifo_descriptors;

/*! Buffer used to hold the streaming response data */
extern uint8_t streaming_resp_buff[];

extern uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX];

extern bool int_pin_usage_native_emulated[COINES_SHUTTLE_PIN_MAX];

bool bhi_streaming_configured = false;

extern void mbuf_user_evt_handler(mbuf_evt_type_t event);

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
/*! Holds the interrupt line state */
static volatile uint8_t stream_feature_line_state[COINES_SHUTTLE_PIN_MAX];

/*! Updated on timer0 expire */
static volatile uint8_t poll_stream_triggered = 0;

/*! Holds active interrupt pin number */
/*lint -e551 */
static uint32_t active_pins = 0;

/*! Used to store the trasmit data */
static uint8_t tx_buf[DECODER_MAX_PKT_SZ] = { 0 };

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/

static int8_t sensor_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    return coines_read_i2c(COINES_I2C_BUS_0, i2c_addr, reg_addr, reg_data, (uint16_t)length);
}

static int8_t sensor_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    return coines_write_i2c(COINES_I2C_BUS_0, i2c_addr, reg_addr, reg_data, (uint16_t)length);
}

static int8_t sensor_spi_read(uint8_t spi_cs, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    return coines_read_spi(COINES_SPI_BUS_0, spi_cs, reg_addr | 0x80, reg_data, (uint16_t)length);
}

static int8_t sensor_spi_write(uint8_t spi_cs, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    return coines_write_spi(COINES_SPI_BUS_0, spi_cs, reg_addr, reg_data, (uint16_t)length);
}

/*!
 *
 * @brief       : API will allocates the buffer for poll/interrupt from RTOS
 *
 * @param[in]   : stream_p - pointer to the stream parameters which contains buffer sizes
 *
 * @return      : None
 */
static void stream_allocate_buffers(stream_descriptor_t *stream_p);

/*!
 *
 * @brief       : API will deallocate the buffer for poll/interrupt from RTOS
 *
 * @param[in]   : stream_p - pointer to the stream parameters which contains buffer sizes
 *
 * @return      : None
 */
static void stream_deallocate_buffers(stream_descriptor_t *stream_p);

/*!
 *
 * @brief       : API will allocates the buffer for fifo streaming from RTOS
 *
 * @param[in]   : fifo_stream_p - pointer to the stream parameters which contains buffer sizes
 *
 * @return      : None
 */
static void stream_allocate_fifo_buffers(stream_fifo_descriptor_t *fifo_stream_p);

/*!
 *
 * @brief       : API will deallocate the buffer for fifo streaming from RTOS
 *
 * @param[in]   : fifo_stream_p - pointer to the stream parameters which contains buffer sizes
 *
 * @return      : None
 */
static void stream_deallocate_fifo_buffers(stream_fifo_descriptor_t *fifo_stream_p);

/*!
 *
 * @brief       : This API will gives the index position at which the read data has to update in response
 *
 * @param[in]   : id - Sensor ID
 *
 * @return      : Position in which read data to be updated
 */
static uint16_t stream_calculate_offset(uint8_t id);

/*!
 *
 * @brief       : This API will be called on polling timer expire
 *
 * @return      : None
 */
static void polling_event_handler(void);

/*!
 *
 * @brief       : This API will be called on data ready interrupt
 *
 * @param[in]   : timestamp in nanosecond
 * @param[in]   : multiio pin
 * @param[in]   : polarity of the multiio pin
 *
 * @return      : None
 */
static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);

/*! Inline function used to free the memory allocated by RTOS */
static void inline  stream_safe_free(uint8_t **p)
{
    if (*p)
    {
        free(*p);
        (*p) = NULL;
    }
}

/**********************************************************************************/
/* functions                                                                      */
/**********************************************************************************/

/*!
 *
 * @brief       : This API will gives the index position at which the read data has to update in response
 */
static uint16_t stream_calculate_offset(uint8_t id)
{
    uint8_t inx;
    uint16_t data_size = 0;

    for (inx = 0; inx < id; inx++)
    {
        data_size += (uint16_t)stream_descriptors[inx].total_data_size;
    }

    return data_size;
}

/*!
 *
 * @brief       : This API will stream the fifo send response
 */
static void streaming_fifo_send_response(stream_fifo_descriptor_t *stream_p)
{
    uint32_t tx_data_size;
    uint8_t *work_p;
    uint8_t packet_index;
    uint32_t size_remaining;
    uint32_t idx;
    uint8_t data_terminating_pos;
    uint32_t pos;

    memset(tx_buf, 0, sizeof(tx_buf));

    tx_data_size = stream_p->number_bytes;

    work_p = streaming_resp_buff;

    memcpy(work_p, stream_p->fifo_data, stream_p->number_bytes);
    work_p += stream_p->number_bytes;

    if (stream_p->intline_count != 0)
    {
        memcpy(work_p, stream_p->DATA_intline, stream_p->intline_count);
    }

    packet_index = 1;
    tx_buf[0] = DECODER_HEADER_VALUE;

    tx_buf[3] = (uint8_t) DECODER_RSP_SUCCESS;    /* Success status */
    tx_buf[4] = DECODER_STRM_FIFO_POLL_RSP_ID;    /* Response identifier for Sync. Data read */
    pos = 0;
    for (idx = 0, size_remaining = tx_data_size; idx < tx_data_size; idx = idx + DECODER_STRM_FIFO_POLL_RSP_PL_SZ)
    {

        if (size_remaining > DECODER_STRM_FIFO_POLL_RSP_PL_SZ)
        {
            tx_buf[2] = packet_index;
            size_remaining -= DECODER_STRM_FIFO_POLL_RSP_PL_SZ;
            tx_buf[1] =
                (uint8_t)(DECODER_STRM_FIFO_POLL_RSP_PL_SZ + DECODER_STRM_FIFO_POLL_RSP_OVERHEAD +
                          DECODER_END_INDICATORS_SIZE);
            data_terminating_pos = DECODER_STRM_FIFO_POLL_RSP_PL_SZ + DECODER_STRM_FIFO_POLL_RSP_OVERHEAD;
            memcpy(&tx_buf[5], &streaming_resp_buff[pos], DECODER_STRM_FIFO_POLL_RSP_PL_SZ);
            pos = pos + DECODER_STRM_FIFO_POLL_RSP_PL_SZ;

        }
        else
        {
            tx_buf[2] = packet_index;
            tx_buf[1] = (uint8_t)(size_remaining + DECODER_STRM_FIFO_POLL_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE);

            /* Update the data in response packet */
            memcpy(&tx_buf[5], &streaming_resp_buff[pos], size_remaining);
            data_terminating_pos = (uint8_t)(size_remaining + DECODER_STRM_FIFO_POLL_RSP_OVERHEAD);
        }

        tx_buf[data_terminating_pos] = DECODER_CR_MACRO;
        tx_buf[data_terminating_pos + 1] = DECODER_LF_MACRO;

        /* transmit to host*/
        write_resp(tx_buf, tx_buf[1]);
        packet_index++;

    }

    memset(streaming_resp_buff, 0, tx_data_size);
}

/*!
 *
 * @brief       : This API will stream the polling send response
 */
static void streaming_polling_send_response(stream_descriptor_t *stream_p)
{
    uint16_t sensor_identifier;
    uint32_t tx_data_size;
    uint8_t *work_p;
    stream_chunkinfo_t *chunk_p;

    uint8_t packet_index;
    uint8_t packetstosend = 0;
    uint8_t data_terminating_pos;
    uint8_t datatosend = 0;
    uint32_t size_remain;

    memset(tx_buf, 0, sizeof(tx_buf));

    sensor_identifier = (uint16_t) (1 << stream_p->channel_id);

    tx_data_size = stream_p->total_data_size;

    work_p = streaming_resp_buff;

    for (uint8_t i = 0; i < stream_p->chunk_count; i++)
    {
        chunk_p = &stream_p->chunks[i];
        memcpy(work_p, chunk_p->DATA_chunk, chunk_p->num_bytes_to_read);
        work_p += chunk_p->num_bytes_to_read;
    }

    if (stream_p->intline_count != 0)
    {
        memcpy(work_p, stream_p->DATA_intline, stream_p->intline_count);
        work_p += stream_p->intline_count;
    }

    /* Adding sensor identifier at the end */
    *work_p++ = (uint8_t) (sensor_identifier >> 8);
    *work_p++ = (uint8_t) (sensor_identifier & 0xFF);
    tx_data_size += 2;

    if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
    {
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 40);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 32);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 24);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 16);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 8);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us);
        tx_data_size += 6;
    }

    if (tx_data_size > DECODER_STRM_POLL_RSP_PL_SZ)
    {
        packetstosend = (uint8_t) ceil((double) tx_data_size / DECODER_STRM_POLL_RSP_PL_SZ);
    }
    else
    {
        packetstosend = 1;
    }

    tx_buf[0] = DECODER_HEADER_VALUE;
    tx_buf[3] = (uint8_t) DECODER_RSP_SUCCESS;    /* Success status */
    tx_buf[4] = DECODER_STRM_POLL_RSP_ID;         /* Response identifier for Sync. Data read */

    packet_index = 1;
    size_remain = tx_data_size;

    while (packetstosend > 0)
    {
        if (size_remain > DECODER_STRM_POLL_RSP_PL_SZ)
        {
            tx_buf[2] = packet_index | 0x80;
            datatosend = DECODER_STRM_POLL_RSP_PL_SZ;
            size_remain -= DECODER_STRM_POLL_RSP_PL_SZ;
            data_terminating_pos = DECODER_STRM_POLL_RSP_PL_SZ + DECODER_STRM_POLL_RSP_OVERHEAD;
        }
        else
        {
            tx_buf[2] = packet_index;
            datatosend = (uint8_t)size_remain;
            data_terminating_pos = (uint8_t)(size_remain + DECODER_STRM_POLL_RSP_OVERHEAD);
        }

        tx_buf[1] = datatosend + DECODER_STRM_POLL_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE;

        memcpy(&tx_buf[5], &streaming_resp_buff[(packet_index - 1) * DECODER_STRM_POLL_RSP_PL_SZ], datatosend);

        tx_buf[data_terminating_pos] = DECODER_CR_MACRO;
        tx_buf[data_terminating_pos + 1] = DECODER_LF_MACRO;

        packetstosend--;
        packet_index++;

        /* transmit to host*/
        write_resp(tx_buf, tx_buf[1]);
    }

    memset(streaming_resp_buff, 0, tx_data_size);
}

#ifdef DMA_STREAMING_SUPPORT
/*!
 *
 * @brief       : This API will stream the dma register using polling method
 */
static void streaming_dma_polling_send_response(stream_descriptor_t *stream_p)
{
    uint16_t dma_identifier;
    uint32_t tx_data_size;
    uint8_t *work_p;
    dma_stream_chunkinfo_t *dma_chunk_p;

    uint8_t packet_index;
    uint8_t packetstosend = 0;
    uint8_t data_terminating_pos;
    uint8_t datatosend = 0;
    uint32_t size_remain;

    memset(tx_buf, 0, sizeof(tx_buf));

    dma_identifier = (uint16_t) (1 << stream_p->channel_id);

    tx_data_size = stream_p->total_dma_data_size;

    work_p = streaming_resp_buff;

    for (uint8_t i = 0; i < stream_p->dma_chunk_count; i++)
    {
        dma_chunk_p = &stream_p->dma_chunks[i];
        memcpy(work_p, dma_chunk_p->data, dma_chunk_p->read_page_len);
        work_p += dma_chunk_p->read_page_len;
    }

    /* Adding sensor identifier at the end */
    *work_p++ = (uint8_t) (dma_identifier >> 8);
    *work_p++ = (uint8_t) (dma_identifier & 0xFF);
    tx_data_size += 2;

    // if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
    // {
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 40);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 32);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 24);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 16);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 8);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us);
    //     tx_data_size += 6;
    // }

    if (tx_data_size > DECODER_STRM_POLL_RSP_PL_SZ)
    {
        packetstosend = (uint8_t) ceil((double) tx_data_size / DECODER_STRM_POLL_RSP_PL_SZ);
    }
    else
    {
        packetstosend = 1;
    }

    tx_buf[0] = DECODER_HEADER_VALUE;
    tx_buf[3] = (uint8_t) DECODER_RSP_SUCCESS;    /* Success status */
    tx_buf[4] = DECODER_STRM_POLL_DMA_RSP_ID;         /* Response identifier for Sync. Data read */

    packet_index = 1;
    size_remain = tx_data_size;

    while (packetstosend > 0)
    {
        if (size_remain > DECODER_STRM_POLL_RSP_PL_SZ)
        {
            tx_buf[2] = packet_index | 0x80;
            datatosend = DECODER_STRM_POLL_RSP_PL_SZ;
            size_remain -= DECODER_STRM_POLL_RSP_PL_SZ;
            data_terminating_pos = DECODER_STRM_POLL_RSP_PL_SZ + DECODER_STRM_POLL_RSP_OVERHEAD;
        }
        else
        {
            tx_buf[2] = packet_index;
            datatosend = (uint8_t)size_remain;
            data_terminating_pos = (uint8_t)(size_remain + DECODER_STRM_POLL_RSP_OVERHEAD);
        }

        tx_buf[1] = datatosend + DECODER_STRM_POLL_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE;

        memcpy(&tx_buf[5], &streaming_resp_buff[(packet_index - 1) * DECODER_STRM_POLL_RSP_PL_SZ], datatosend);

        tx_buf[data_terminating_pos] = DECODER_CR_MACRO;
        tx_buf[data_terminating_pos + 1] = DECODER_LF_MACRO;

        packetstosend--;
        packet_index++;

        /* transmit to host*/
        write_resp(tx_buf, tx_buf[1]);
    }

    memset(streaming_resp_buff, 0, tx_data_size);
}

/*!
 *
 * @brief       : This API will stream the dma register using polling method
 */
static void streaming_dma_interrupt_send_response(stream_descriptor_t *stream_p)
{
    uint16_t dma_identifier;
    uint32_t tx_data_size;
    uint8_t *work_p;
    dma_stream_chunkinfo_t *dma_chunk_p;

    uint8_t packet_index;
    uint8_t packetstosend = 0;
    uint8_t data_terminating_pos;
    uint8_t datatosend = 0;
    uint32_t size_remain;

    memset(tx_buf, 0, sizeof(tx_buf));

    dma_identifier = (uint16_t) (1 << stream_p->channel_id);

    tx_data_size = stream_p->total_dma_data_size;

    work_p = streaming_resp_buff;

    for (uint8_t i = 0; i < stream_p->dma_chunk_count; i++)
    {
        dma_chunk_p = &stream_p->dma_chunks[i];
        memcpy(work_p, dma_chunk_p->data, dma_chunk_p->read_page_len);
        work_p += dma_chunk_p->read_page_len;
    }

    /* Adding sensor identifier at the end */
    *work_p++ = (uint8_t) (dma_identifier >> 8);
    *work_p++ = (uint8_t) (dma_identifier & 0xFF);
    tx_data_size += 2;

    // if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
    // {
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 40);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 32);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 24);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 16);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 8);
    //     *work_p++ = (uint8_t) (stream_p->packet_timestamp_us);
    //     tx_data_size += 6;
    // }

    if (tx_data_size > DECODER_STRM_POLL_RSP_PL_SZ)
    {
        packetstosend = (uint8_t) ceil((double) tx_data_size / DECODER_STRM_POLL_RSP_PL_SZ);
    }
    else
    {
        packetstosend = 1;
    }

    tx_buf[0] = DECODER_HEADER_VALUE;
    tx_buf[3] = (uint8_t) DECODER_RSP_SUCCESS;    /* Success status */
    tx_buf[4] = DECODER_STRM_INT_DMA_RSP_ID;         /* Response identifier for Sync. Data read */

    packet_index = 1;
    size_remain = tx_data_size;

    while (packetstosend > 0)
    {
        if (size_remain > DECODER_STRM_POLL_RSP_PL_SZ)
        {
            tx_buf[2] = packet_index | 0x80;
            datatosend = DECODER_STRM_POLL_RSP_PL_SZ;
            size_remain -= DECODER_STRM_POLL_RSP_PL_SZ;
            data_terminating_pos = DECODER_STRM_POLL_RSP_PL_SZ + DECODER_STRM_POLL_RSP_OVERHEAD;
        }
        else
        {
            tx_buf[2] = packet_index;
            datatosend = (uint8_t)size_remain;
            data_terminating_pos = (uint8_t)(size_remain + DECODER_STRM_POLL_RSP_OVERHEAD);
        }

        tx_buf[1] = datatosend + DECODER_STRM_POLL_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE;

        memcpy(&tx_buf[5], &streaming_resp_buff[(packet_index - 1) * DECODER_STRM_POLL_RSP_PL_SZ], datatosend);

        tx_buf[data_terminating_pos] = DECODER_CR_MACRO;
        tx_buf[data_terminating_pos + 1] = DECODER_LF_MACRO;

        packetstosend--;
        packet_index++;

        /* transmit to host*/
        write_resp(tx_buf, tx_buf[1]);
    }

    memset(streaming_resp_buff, 0, tx_data_size);
}
#endif /* DMA_STREAMING_SUPPORT */

/*!
 *
 * @brief       : This API will stream the polling old send response
 */
static void streaming_polling_old_send_response(uint16_t identifier)
{
    uint8_t *work_p = streaming_resp_buff;
    stream_chunkinfo_t *chunk_p;
    uint8_t idx;
    stream_descriptor_t *stream_p = stream_descriptors;
    uint16_t total_data_size = 0;
    uint16_t size_remaining;
    uint8_t packet_index;
    uint8_t data_terminating_pos = 0;

    uint16_t pos = 0;

    memset(tx_buf, 0, sizeof(tx_buf));
    for (idx = 0; idx < stream_active_count;)
    {
        if (((identifier) & (1 << idx)) != 0)
        {
            stream_p = &stream_descriptors[idx];
            pos = stream_calculate_offset(idx);

            work_p = &streaming_resp_buff[pos];

            for (uint8_t i = 0; i < stream_p->chunk_count; i++)
            {
                chunk_p = &stream_p->chunks[i];
                if (chunk_p->num_bytes_to_read != 0)
                {
                    memcpy(work_p, chunk_p->DATA_chunk, chunk_p->num_bytes_to_read);
                    work_p += chunk_p->num_bytes_to_read;
                }
            }

            if (stream_p->intline_count != 0)
            {
                memcpy(work_p, stream_p->DATA_intline, stream_p->intline_count);
                work_p += stream_p->intline_count;
            }
        }

        total_data_size += (uint16_t)stream_descriptors[idx].total_data_size;
        idx++;

    }

    if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
    {
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 40);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 32);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 24);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 16);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 8);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us);

        total_data_size += 6;
    }

    packet_index = 1;
    tx_buf[0] = DECODER_HEADER_VALUE;

    tx_buf[3] = (uint8_t) DECODER_RSP_SUCCESS;    /* Success status */
    tx_buf[4] = DECODER_STRM_OLD_POLL_RSP_ID;     /* Response identifier for Sync. Data read */
    pos = 0;
    for (idx = 0, size_remaining = total_data_size; idx < total_data_size; idx = idx + DECODER_STRM_POLL_RSP_PL_SZ)
    {

        if (size_remaining > DECODER_STRM_POLL_RSP_PL_SZ)
        {
            tx_buf[2] = packet_index;
            size_remaining -= DECODER_STRM_POLL_RSP_PL_SZ;
            tx_buf[1] =
                (uint8_t)(DECODER_STRM_POLL_RSP_PL_SZ + DECODER_STRM_POLL_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE);
            data_terminating_pos = DECODER_STRM_POLL_RSP_PL_SZ + DECODER_STRM_POLL_RSP_OVERHEAD;
            memcpy(&tx_buf[5], &streaming_resp_buff[pos], DECODER_STRM_POLL_RSP_PL_SZ);
            pos = pos + DECODER_STRM_POLL_RSP_PL_SZ;

        }
        else
        {
            tx_buf[2] = packet_index;
            tx_buf[1] = (uint8_t)(size_remaining + DECODER_STRM_POLL_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE);

            /* Update the data in response packet */
            memcpy(&tx_buf[5], &streaming_resp_buff[pos], size_remaining);
            data_terminating_pos = (uint8_t)(size_remaining + DECODER_STRM_POLL_RSP_OVERHEAD);
        }

        tx_buf[data_terminating_pos] = DECODER_CR_MACRO;
        tx_buf[data_terminating_pos + 1] = DECODER_LF_MACRO;

        /* transmit to host*/
        write_resp(tx_buf, tx_buf[1]);
        packet_index++;

    }

    memset(streaming_resp_buff, 0, total_data_size);
}

/*!
 *
 * @brief       : This API will stream the single fifo send response
 */
static void streaming_single_fifo_send_response(stream_descriptor_t *stream_p, uint16_t read_bytes, uint8_t last_frame)
{
    int32_t bytes_transmitted = 0;
    int32_t data_available;
    static uint8_t residue_buffer[100] = { 0 };
    uint8_t datalength = 0;
    static uint8_t residue_len = 0;
    uint8_t *work_p;
    uint8_t terminator_pos;

    memset(tx_buf, 0, sizeof(tx_buf));

    work_p = streaming_resp_buff;

    /* Copy residue data if any to FIFO data array.*/
    if (residue_len > 0)
    {
        memcpy(work_p, residue_buffer, residue_len);

        /* Update the data length*/
        datalength = residue_len;
        work_p += datalength;
        residue_len = 0; /* Reset the residue length*/
    }

    /* Copy the read data i.e., individual frame into FIFO data array*/
    memcpy(work_p, stream_p->chunks[1].DATA_chunk, read_bytes);

    /* Update the data length*/
    datalength += (uint8_t)read_bytes;

    /* Copy of FIFO data length is taken for transmit operation */
    data_available = datalength;

    /* Header is updated only once. */
    tx_buf[0] = DECODER_HEADER_VALUE;
    tx_buf[2] = 1;

    /* Success status */
    tx_buf[3] = 0x00;

    /* Response identifier for Sync. Data read */
    tx_buf[4] = DECODER_STRM_INT_RSP_ID;

    /* Sensor ID as given by the host */
    tx_buf[5] = stream_p->channel_id;

    /* Data packet number. */
    tx_buf[6] = (uint8_t) (stream_p->data_packet_counter >> 24);
    tx_buf[7] = (uint8_t) (stream_p->data_packet_counter >> 16);
    tx_buf[8] = (uint8_t) (stream_p->data_packet_counter >> 8);
    tx_buf[9] = (uint8_t) (stream_p->data_packet_counter++);

    /* Dummy timestamp values Where it is used need to find out?*/
    tx_buf[10] = 0;
    tx_buf[11] = 0;
    tx_buf[12] = 0;
    tx_buf[13] = 0;

    /*
     * Loop is implemented to transmit the FIFO data.
     * If FIFO data array contains more than MAXIMUM_DATA_IN_PACKET, slice by MAXIMUM_DATA_IN_PACKET and send to host.
     */

    while (data_available > DECODER_STRM_FIFO_RSP_PL_SZ)
    {
        /* Copy the data to transmit buffer*/
        memcpy(&tx_buf[14], &streaming_resp_buff[bytes_transmitted], DECODER_STRM_FIFO_RSP_PL_SZ);
        data_available -= DECODER_STRM_FIFO_RSP_PL_SZ;
        bytes_transmitted += DECODER_STRM_FIFO_RSP_PL_SZ;
        terminator_pos = (uint8_t) bytes_transmitted + DECODER_STRM_FIFO_RSP_OVERHEAD;
        tx_buf[terminator_pos] = DECODER_CR_MACRO;
        tx_buf[terminator_pos + 1] = DECODER_LF_MACRO;
        tx_buf[1] = (uint8_t) bytes_transmitted + DECODER_STRM_FIFO_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE; /* Size field
                                                                                                       * updated*/

        write_resp(tx_buf, tx_buf[1]);
    }

    /* Incase of last frame, how much ever data left in FIFO data array needs to be transmitted. */
    if (last_frame)
    {
        /* Copy the data to transmit buffer*/
        memcpy(&tx_buf[14], &streaming_resp_buff[bytes_transmitted], (uint32_t)data_available);
        terminator_pos = (uint8_t) data_available + DECODER_STRM_FIFO_RSP_OVERHEAD;
        tx_buf[terminator_pos] = DECODER_CR_MACRO;
        tx_buf[terminator_pos + 1] = DECODER_LF_MACRO;
        tx_buf[1] = (uint8_t) data_available + DECODER_STRM_FIFO_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE;/* Size field
                                                                                                           * updated*/

        write_resp(tx_buf, tx_buf[1]);

        residue_len = 0;
    }
    else
    {
        /* Copy available data in FIFO data array into Residue array. */
        memcpy(&residue_buffer[0], &streaming_resp_buff[bytes_transmitted], (uint32_t)(datalength - bytes_transmitted));
        residue_len = (uint8_t)(datalength - bytes_transmitted);
    }
}

/*!
 *
 * @brief       : This API will stream the multiple fifo send response
 */
static void streaming_multiple_fifo_send_response(stream_descriptor_t *stream_p, uint8_t chunk_id, uint16_t read_bytes)
{
    uint8_t packetstosend, packetnumber;
    uint32_t datatosend, terminator_pos, tx_data_size;
    uint8_t *work_p;
    stream_chunkinfo_t *chunk_p;
    uint8_t fifo_identifier;

    tx_data_size = read_bytes;

    memset(tx_buf, 0, sizeof(tx_buf));

    work_p = streaming_resp_buff;

    /* Copy the data from the chunk*/
    chunk_p = &stream_p->chunks[chunk_id];
    memcpy(work_p, chunk_p->DATA_chunk, read_bytes);
    work_p += read_bytes;

    /* Calculate the FIFO identifier*/
    fifo_identifier = (chunk_id - 2) / 2;

    if (stream_p->intline_count != 0)
    {
        memcpy(work_p, stream_p->DATA_intline, stream_p->intline_count);
    }

    if (tx_data_size > DECODER_STRM_FIFO_RSP_PL_SZ)
    {
        packetstosend = (uint8_t) ceil((double) tx_data_size / DECODER_STRM_FIFO_RSP_PL_SZ);
    }
    else
    {
        packetstosend = 1;
    }

    tx_buf[0] = DECODER_HEADER_VALUE;

    packetnumber = 1;
    while (packetstosend > 0)
    {
        /* Write packet number */
        tx_buf[2] = packetnumber;
        if (packetstosend > 1)
        {
            /* set a flag for all packets other than the last */
            tx_buf[2] |= 0x80;
        }

        tx_buf[3] = (uint8_t) DECODER_RSP_SUCCESS;    /* Success status */
        tx_buf[4] = DECODER_STRM_INT_RSP_ID;          /* Response identifier for Sync. Data read */
        tx_buf[5] = fifo_identifier + 1;              /* FIFO identifier. */

        /* Data packet number.*/
        tx_buf[6] = (uint8_t) ((stream_p->data_packet_counter & 0xff000000) >> 24);
        tx_buf[7] = (uint8_t) ((stream_p->data_packet_counter & 0x00ff0000) >> 16);
        tx_buf[8] = (uint8_t) ((stream_p->data_packet_counter & 0x0000ff00) >> 8);
        tx_buf[9] = (uint8_t) (stream_p->data_packet_counter & 0x000000ff);

        /* Dummy timestamp value to conform protocol protocol response format*/
        tx_buf[10] = 0;
        tx_buf[11] = 0;
        tx_buf[12] = 0;
        tx_buf[13] = 0;

        stream_p->data_packet_counter++;

        if (tx_data_size > DECODER_STRM_FIFO_RSP_PL_SZ)
        {
            datatosend = DECODER_STRM_FIFO_RSP_PL_SZ;
            tx_data_size -= DECODER_STRM_FIFO_RSP_PL_SZ;
        }
        else
        {
            datatosend = tx_data_size;
        }

        /* Update the data in response packet */
        memcpy(&tx_buf[14], &streaming_resp_buff[(packetnumber - 1) * DECODER_STRM_FIFO_RSP_PL_SZ], datatosend);

        /* Add terminator sequence */
        terminator_pos = datatosend + DECODER_STRM_FIFO_RSP_OVERHEAD;
        tx_buf[terminator_pos] = DECODER_CR_MACRO;
        tx_buf[terminator_pos + 1] = DECODER_LF_MACRO;

        /* Update the package size */
        tx_buf[1] = (uint8_t) (datatosend + DECODER_STRM_FIFO_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE);

        /* transmit to host */
        write_resp(tx_buf, tx_buf[1]);

        packetstosend--;
        packetnumber++;
    }
}

/*!
 *
 * @brief       :This API will stream the interrupt send response
 */
static void streaming_interrupt_send_response(stream_descriptor_t *stream_p)
{
    uint8_t packetstosend, packetnumber;
    uint32_t datatosend, terminator_pos, tx_data_size;
    uint8_t *work_p;

    stream_chunkinfo_t *chunk_p;

    memset(tx_buf, 0, sizeof(tx_buf));

    tx_data_size = stream_p->total_data_size;

    work_p = streaming_resp_buff;
    for (uint8_t i = 0; i < stream_p->chunk_count; i++)
    {
        chunk_p = &stream_p->chunks[i];
        memcpy(work_p, chunk_p->DATA_chunk, chunk_p->num_bytes_to_read);
        work_p += chunk_p->num_bytes_to_read;
    }

    if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
    {
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 40);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 32);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 24);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 16);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us >> 8);
        *work_p++ = (uint8_t) (stream_p->packet_timestamp_us);

        tx_data_size += 6;
    }

    if (stream_p->intline_count != 0)
    {
        memcpy(work_p, stream_p->DATA_intline, stream_p->intline_count);
    }

    if (tx_data_size > DECODER_STRM_INT_RSP_PL_SZ)
    {
        packetstosend = (uint8_t) ceil((double) tx_data_size / DECODER_STRM_INT_RSP_PL_SZ);
    }
    else
    {
        packetstosend = 1;
    }

    tx_buf[0] = DECODER_HEADER_VALUE;

    packetnumber = 1;
    while (packetstosend > 0)
    {
        /* Write packet number */
        tx_buf[2] = packetnumber;
        if (packetstosend > 1)
        {
            /* set a flag for all packets other than the last */
            tx_buf[2] |= 0x80;
        }

        tx_buf[3] = (uint8_t) DECODER_RSP_SUCCESS;  /* Success status */
        tx_buf[4] = DECODER_STRM_INT_RSP_ID;        /* Response identifier for Sync. Data read */
        tx_buf[5] = stream_p->channel_id;           /* Sensor ID as given by the host */

        /* Data packet number. */
        tx_buf[6] = (uint8_t) ((stream_p->data_packet_counter & 0xff000000) >> 24);
        tx_buf[7] = (uint8_t) ((stream_p->data_packet_counter & 0x00ff0000) >> 16);
        tx_buf[8] = (uint8_t) ((stream_p->data_packet_counter & 0x0000ff00) >> 8);
        tx_buf[9] = (uint8_t) (stream_p->data_packet_counter & 0x000000ff);
        stream_p->data_packet_counter++;

        if (tx_data_size > DECODER_STRM_INT_RSP_PL_SZ)
        {
            datatosend = DECODER_STRM_INT_RSP_PL_SZ;
            tx_data_size -= DECODER_STRM_INT_RSP_PL_SZ;
        }
        else
        {
            datatosend = tx_data_size;
        }

        /* Update the data in response packet */
        memcpy(&tx_buf[10], &streaming_resp_buff[(packetnumber - 1) * DECODER_STRM_INT_RSP_PL_SZ], datatosend);

        /* Add terminator sequence */
        terminator_pos = datatosend + DECODER_STRM_INT_RSP_OVERHEAD;
        tx_buf[terminator_pos] = DECODER_CR_MACRO;
        tx_buf[terminator_pos + 1] = DECODER_LF_MACRO;

        /* Update the package size */
        tx_buf[1] = (uint8_t) (datatosend + DECODER_STRM_INT_RSP_OVERHEAD + DECODER_END_INDICATORS_SIZE);

        /* Add to buffer */
        mbuf_add_to_buffer(tx_buf, tx_buf[1]);

        packetstosend--;
        packetnumber++;
    }
}

/*!
 * @brief   This API is used to read the data and Int states for one stream channel
 *
 */
static int8_t read_fifo_polling_sample(stream_fifo_descriptor_t *stream_p)
{
    int8_t ret = 0;
    uint8_t interrupt_line;
    enum coines_pin_value gpio_pin_val = COINES_PIN_VALUE_LOW;
    enum coines_pin_direction gpio_dir;

    /* read feature interrupts state for current stream */
    if (stream_p->intline_info != NULL)
    {
        /* Get the Interrupt line information and update them in the buffer for transmission */
        for (uint8_t int_idx = 0; int_idx < stream_p->intline_count; int_idx++)
        {
            interrupt_line = stream_p->intline_info[int_idx];

            coines_get_pin_config((enum coines_multi_io_pin)interrupt_line, &gpio_dir, &gpio_pin_val);

            if (stream_feature_line_state[interrupt_line] == 1)
            {
                /* the interrupt was triggered, so mark it when sending to host and update the line status according
                 * with the current GPIO value */
                stream_p->DATA_intline[int_idx] = 1;
                stream_feature_line_state[interrupt_line] = (gpio_pin_val == COINES_PIN_VALUE_HIGH) ? 1 : 0;
            }
            else
            {
                /* the interrupt was not triggered, so update the line status according with the current GPIO value, and
                 * send this new value to the host */
                stream_feature_line_state[interrupt_line] = (gpio_pin_val == COINES_PIN_VALUE_HIGH) ? 1 : 0;
                stream_p->DATA_intline[int_idx] = stream_feature_line_state[interrupt_line];
            }
        }
    }

    if (stream_p->feature == STREAM_FIFO_BURST_READ)
    {
        if (stream_p->param_interface == STREAM_IF_I2C)
        {
            ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                                  stream_p->fifo_reg_address,
                                  stream_p->fifo_data,
                                  stream_p->number_bytes);
        }
        else
        {

            ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                  stream_p->fifo_reg_address,
                                  stream_p->fifo_data,
                                  stream_p->number_bytes);

        }
    }
    else
    {
        uint16_t inx = 0;
        uint16_t num_times_read = stream_p->number_bytes / stream_p->fifo_frame_size;
        uint16_t data_pos = 0;
        uint16_t read_bytes;
        uint8_t reminder_flag = 0;

        if ((stream_p->number_bytes % stream_p->fifo_frame_size) != 0)
        {

            reminder_flag = 1;
        }

        read_bytes = stream_p->fifo_frame_size;
        for (; inx < num_times_read;)
        {

            if (stream_p->param_interface == STREAM_IF_I2C)
            {
                ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                                      stream_p->fifo_reg_address,
                                      stream_p->fifo_data + data_pos,
                                      read_bytes);
            }
            else
            {

                ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                      stream_p->fifo_reg_address,
                                      stream_p->fifo_data + data_pos,
                                      read_bytes);
            }

            data_pos += stream_p->fifo_frame_size;
            inx++;
        }

        if (reminder_flag == 1)
        {
            read_bytes = stream_p->number_bytes % stream_p->fifo_frame_size;
            if (stream_p->param_interface == STREAM_IF_I2C)
            {
                ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                                      stream_p->fifo_reg_address,
                                      stream_p->fifo_data + data_pos,
                                      read_bytes);
            }
            else
            {

                ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                      stream_p->fifo_reg_address,
                                      stream_p->fifo_data + data_pos,
                                      read_bytes);

            }
        }
    }

    return ret;
}

/*!
 * @brief   This API is used to read and send single fifo data
 *
 */
static int8_t read_and_send_single_fifo_data(stream_descriptor_t *stream_p)
{
    uint16_t size_to_read = 0;
    uint8_t chunk_len_idx;
    uint16_t read_bytes;
    uint8_t last_frame = 0;
    int8_t ret = 0;

    /* Read the length*/
    stream_chunkinfo_t *read_len_p = &stream_p->chunks[0];

    read_len_p->DATA_chunk = (uint8_t*) stream_memory_alloc(stream_p->chunks[0].num_bytes_to_read * sizeof(uint8_t));

    if (stream_p->param_interface == STREAM_IF_I2C)
    {
        ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                              read_len_p->startaddress,
                              read_len_p->DATA_chunk,
                              read_len_p->num_bytes_to_read);
    }
    else
    {
        if (read_len_p->num_bytes_to_read != 0)
        {
            ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                  read_len_p->startaddress,
                                  read_len_p->DATA_chunk,
                                  read_len_p->num_bytes_to_read);

        }
    }

    /* Get the size of the data to be read*/
    for (chunk_len_idx = 0; chunk_len_idx < read_len_p->num_bytes_to_read; chunk_len_idx++)
    {
        size_to_read |= (uint16_t)(read_len_p->DATA_chunk[chunk_len_idx] << (chunk_len_idx * 8));
    }

    stream_chunkinfo_t *data_p = &stream_p->chunks[1];

    data_p->DATA_chunk = (uint8_t*) stream_memory_alloc(stream_p->framelength * sizeof(uint8_t));

    data_p->num_bytes_to_read = size_to_read;

    while (size_to_read > 0)
    {

        if (size_to_read > stream_p->framelength)
        {
            read_bytes = stream_p->framelength;
        }
        else
        {
            read_bytes = size_to_read;
        }

        if (size_to_read <= stream_p->framelength)
        {
            last_frame = 1;
        }

        if (stream_p->param_interface == STREAM_IF_I2C)
        {
            ret =
                sensor_i2c_read((uint8_t) stream_p->dev_address, data_p->startaddress, data_p->DATA_chunk, read_bytes);
        }
        else
        {
            ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                  data_p->startaddress,
                                  data_p->DATA_chunk,
                                  read_bytes);

        }

        size_to_read -= read_bytes;

        /* Send the data to host*/
        streaming_single_fifo_send_response(stream_p, read_bytes, last_frame);
    }

    stream_safe_free(&data_p->DATA_chunk);

    stream_safe_free(&read_len_p->DATA_chunk);

    return ret;
}

/*!
 * @brief   This API is used to read and send multiple fifo data
 *
 */
static int8_t read_and_send_multiple_fifo_data(stream_descriptor_t *stream_p)
{
    uint8_t interrupt_fifo_status;
    uint8_t num_interrupt_mask;
    uint8_t idx;
    uint8_t len_chunk_index;
    int16_t size_to_read = 0;
    uint8_t chunk_len_idx = 0;
    uint8_t data_chunk_index;
    uint16_t read_bytes;
    uint16_t data_pos_index = 0;
    int8_t ret = 0;

    /*TODO: implement */

    /* Get the number of Interrupt mask in the command*/
    /* Interrupt status + Number of bytes to read*/
    /* Wakeup FIFO Size register1 + Number of bytes to read */
    /* Wakeup FIFO Data register 1+ Number of bytes to read from the previous read of size register1 */
    /* Non-Wakeup FIFO Size register2 + Number of bytes to read */
    /* Non-Wakeup FIFO Data register 2+ Number of bytes to read from the previous read of size register2 */
    /* In this interrupt mask is applicable only to the data register*/

    /* In our example number of chunks is 5, we implement this following formula to calcuate the number of interrupt
     * mask is available */
    num_interrupt_mask = (stream_p->chunk_count - 1) / 2;

    /* Read the interrupt status*/
    stream_chunkinfo_t *fifo_identification_p = &stream_p->chunks[0];

    if (stream_p->param_interface == STREAM_IF_I2C)
    {

        ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                              fifo_identification_p->startaddress,
                              fifo_identification_p->DATA_chunk,
                              fifo_identification_p->num_bytes_to_read);
    }
    else
    {
        if (fifo_identification_p->num_bytes_to_read != 0)
        {
            ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                  fifo_identification_p->startaddress,
                                  fifo_identification_p->DATA_chunk,
                                  fifo_identification_p->num_bytes_to_read);

        }
    }

    /* Get the interrupt fifo status*/
    interrupt_fifo_status = fifo_identification_p->DATA_chunk[0];

    for (idx = 0; idx < num_interrupt_mask; idx++)
    {
        if ((interrupt_fifo_status & stream_p->chunk_mask[idx]) != 0)
        {
            len_chunk_index = (uint8_t)(1 + (2 * idx));

            /* Read the length*/
            stream_chunkinfo_t *read_len_p = &stream_p->chunks[len_chunk_index];

            read_len_p->DATA_chunk = (uint8_t*) stream_memory_alloc(
                stream_p->chunks[len_chunk_index].num_bytes_to_read * sizeof(uint8_t));

            if (stream_p->param_interface == STREAM_IF_I2C)
            {
                ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                                      read_len_p->startaddress,
                                      read_len_p->DATA_chunk,
                                      read_len_p->num_bytes_to_read);
            }
            else
            {
                if (read_len_p->num_bytes_to_read != 0)
                {
                    ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                          read_len_p->startaddress,
                                          read_len_p->DATA_chunk,
                                          read_len_p->num_bytes_to_read);

                }
            }

            /* Get the size of the data to be read*/
            for (chunk_len_idx = 0; chunk_len_idx < read_len_p->num_bytes_to_read; chunk_len_idx++)
            {
                size_to_read |= (int16_t)(read_len_p->DATA_chunk[chunk_len_idx] << (chunk_len_idx * 8));
            }

            if (size_to_read > 0)
            {
                data_chunk_index = (uint8_t)(2 + (2 * idx));

                /* Read the framelength*/
                if (size_to_read > (int16_t)stream_p->framelength)
                {
                    read_bytes = stream_p->framelength;
                }
                else
                {
                    read_bytes = (uint16_t)size_to_read;
                }

                stream_chunkinfo_t *data_p = &stream_p->chunks[data_chunk_index];

                data_p->DATA_chunk = (uint8_t*) stream_memory_alloc(read_bytes * sizeof(uint8_t));

                while (size_to_read > 0)
                {
                    if (stream_p->param_interface == STREAM_IF_I2C)
                    {
                        ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                                              data_p->startaddress,
                                              &data_p->DATA_chunk[data_pos_index],
                                              read_bytes);
                    }
                    else
                    {
                        ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                              data_p->startaddress,
                                              &data_p->DATA_chunk[data_pos_index],
                                              read_bytes);
                    }

                    size_to_read -= (int16_t)read_bytes;

                    /* Send the data to host*/
                    streaming_multiple_fifo_send_response(stream_p, data_chunk_index, read_bytes);

                }

                stream_safe_free(&data_p->DATA_chunk);
            }

            size_to_read = 0;
            stream_safe_free(&read_len_p->DATA_chunk);
        }
    }

    return ret;
}

/*!
 * @brief This function is used to read the data and Int states for one stream channel
 *
 */
static int8_t read_stream_sample(stream_descriptor_t *stream_p)
{
    /*
     * TODO: turn this to a non-blocking read, with a post-processor handling sending the data
     * TODO: implement in-place reading, directly inside the unified buffer, and not in the chunk data
     */
    int8_t ret = 0;
    int16_t pin_status;
    enum coines_pin_value gpio_pin_val = COINES_PIN_VALUE_LOW;
    enum coines_pin_direction gpio_dir;

    /* read feature interrupts state for current stream */
    if ((stream_p->intline_info != NULL) && (stream_p->mode == STREAM_MODE_POLLING))
    {
        /* Get the Interrupt line information and update them in the buffer for transmission */
        for (uint8_t int_idx = 0; int_idx < stream_p->intline_count; int_idx++)
        {
            if (stream_feature_line_state[stream_p->intline_info[int_idx]] == 1)
            {
                if (stream_p->hw_pin_state == 0)/*active low*/
                {
                    stream_p->DATA_intline[int_idx] = 0;
                }
                else
                {
                    stream_p->DATA_intline[int_idx] = 1;
                }

                stream_feature_line_state[stream_p->intline_info[int_idx]] = 0;
            }
            else
            {
                pin_status = coines_get_pin_config((enum coines_multi_io_pin)stream_p->intline_info[int_idx],
                                                   &gpio_dir,
                                                   &gpio_pin_val);
                if (pin_status == COINES_SUCCESS)
                {
                    stream_p->DATA_intline[int_idx] = (uint8_t) gpio_pin_val;
                }
                else
                {
                    if (stream_p->hw_pin_state == 0) /*active low*/
                    {
                        stream_p->DATA_intline[int_idx] = 1;
                    }
                    else
                    {
                        stream_p->DATA_intline[int_idx] = 0;
                    }
                }
            }
        }
    }

    if ((stream_p->read_mode == STREAM_READ_N_CHUNKS) || (stream_p->read_mode == STREAM_READ_2_CHUNKS_POLLING))
    {
        if (stream_p->param_interface == STREAM_IF_I2C)
        {
            if (stream_p->clear_on_write)
            {
                /* Dummy byte information also read based on the input */
                ret = sensor_i2c_read((uint8_t)stream_p->dev_address,
                                      stream_p->reg_info.startaddress,
                                      stream_p->reg_info.data_buf,
                                      (stream_p->reg_info.num_bytes_to_clear + stream_p->reg_info.dummy_byte));
            }

            for (uint8_t i = 0; i < stream_p->chunk_count; i++)
            {
                stream_chunkinfo_t *chunk_p = &stream_p->chunks[i];
                ret = sensor_i2c_read((uint8_t) stream_p->dev_address,
                                      chunk_p->startaddress,
                                      chunk_p->DATA_chunk,
                                      chunk_p->num_bytes_to_read);
            }

            if ((stream_p->clear_on_write) && (COINES_SUCCESS == ret))
            {
                /* Ignoring dummy byte if present and writing to register for clearing status register */
                ret = sensor_i2c_write((uint8_t)stream_p->dev_address,
                                       stream_p->reg_info.startaddress,
                                       &stream_p->reg_info.data_buf[stream_p->reg_info.dummy_byte],
                                       stream_p->reg_info.num_bytes_to_clear);
            }
        }
        else
        {
            if (stream_p->clear_on_write)
            {
                /* Dummy byte information also read based on the input */
                ret = sensor_spi_read((uint8_t)stream_p->param_interface,
                                      stream_p->reg_info.startaddress,
                                      stream_p->reg_info.data_buf,
                                      (stream_p->reg_info.num_bytes_to_clear + stream_p->reg_info.dummy_byte));
            }

            for (uint8_t i = 0; i < stream_p->chunk_count; i++)
            {
                stream_chunkinfo_t *chunk_p = &stream_p->chunks[i];
                ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                      chunk_p->startaddress,
                                      chunk_p->DATA_chunk,
                                      chunk_p->num_bytes_to_read);
            }

            if ((stream_p->clear_on_write) && (COINES_SUCCESS == ret))
            {
                /* Ignoring dummy byte if present and writing to register for clearing status register */
                ret = sensor_spi_write((uint8_t)stream_p->param_interface,
                                       stream_p->reg_info.startaddress,
                                       &stream_p->reg_info.data_buf[stream_p->reg_info.dummy_byte],
                                       stream_p->reg_info.num_bytes_to_clear);
            }
        }

        if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
        {
            stream_p->packet_timestamp_us = coines_get_micro_sec();
        }
    }
    else
    {
        ret = DECODER_RET_COMM_FAILED;
    }

    return ret;
}

#ifdef DMA_STREAMING_SUPPORT
/*!
 * @brief This function is used to read the dma register data for one stream channel
 *
 */
static int8_t read_dma_stream_sample(stream_descriptor_t *stream_p)
{
    int8_t ret = 0;
    
    if ((stream_p->read_mode == STREAM_READ_N_CHUNKS) || (stream_p->read_mode == STREAM_READ_2_CHUNKS_POLLING))
    {
        if (stream_p->param_interface == STREAM_IF_I2C)
        {
            for (uint8_t i = 0; i < stream_p->dma_chunk_count; i++)
            {
                dma_stream_chunkinfo_t *dma_chunk_p = &stream_p->dma_chunks[i];
                ret = sensor_i2c_write((uint8_t)stream_p->dev_address,
                                       dma_chunk_p->write_page_address,
                                       &dma_chunk_p->write_page_number,
                                       DMA_WRITE_REG_LEN);

                                       
                if (COINES_SUCCESS == ret)
                {
                    ret = sensor_i2c_read((uint8_t)stream_p->dev_address,
                                        dma_chunk_p->read_page_address,
                                        dma_chunk_p->data,
                                        dma_chunk_p->read_page_len);
                }

            }
            
        }
        else
        {
            for (uint8_t i = 0; i < stream_p->dma_chunk_count; i++)
            {
                dma_stream_chunkinfo_t *dma_chunk_p = &stream_p->dma_chunks[i];
                ret = sensor_spi_write((uint8_t)stream_p->param_interface,
                                        dma_chunk_p->write_page_address,
                                        &dma_chunk_p->write_page_number,
                                        DMA_WRITE_REG_LEN);
                

                if(ret == COINES_SUCCESS)
                {
                    ret = sensor_spi_read((uint8_t) stream_p->param_interface,
                                        dma_chunk_p->read_page_address,
                                        dma_chunk_p->data,
                                        dma_chunk_p->read_page_len);
                }
            }
        }

        if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
        {
            stream_p->packet_timestamp_us = coines_get_micro_sec();
        }
    }
    else
    {
        ret = DECODER_RET_COMM_FAILED;
    }

    return ret;
}
#endif /* DMA_STREAMING_SUPPORT */

/*!
 * @brief This function read interrupts
 */
void read_interrupt_line_state(uint8_t multi_io)
{
    stream_descriptor_t *stream_p;
    enum coines_pin_value gpio_pin_val = COINES_PIN_VALUE_LOW;
    enum coines_pin_direction gpio_dir;

    for (uint8_t int_idx = 0; int_idx < stream_active_count; int_idx++)
    {
        stream_p = &stream_descriptors[int_idx];
        if (stream_p->data_ready_int == multi_io)
        {
            if (stream_p->intline_info != NULL)
            {
                for (uint8_t int_line = 0; int_line < stream_p->intline_count; int_line++)
                {
                	coines_get_pin_config((enum coines_multi_io_pin)stream_p->intline_info[int_line],
                			&gpio_dir,
							&gpio_pin_val);

                    stream_p->DATA_intline[int_line] = (uint8_t) gpio_pin_val;
                }
            }

            active_pins |= 1 << multi_io;
        }
    }
}

/*!
 * @brief This function handles interrupts associated with interrupt streaming mode
 *
 * @param[in] timestamp: timestamp in nanosecond
 * @param[in] multiio_pin : multiio pin number that triggered the interrupt
 * @param[in] multiio_pin_polarity : polarity of the active state of this multiio pin
 *
 * @return None
 * @retval None
 */

static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin_polarity;
    job_data_t job_data = { 0 };

    job_data.multiio_pin = multiio_pin;
    job_data.timestamp_us = timestamp / 1000;

    if (job_queue_add_job(stream_interrupt_data_acq, (uint8_t *)&job_data, true) != JOB_QUEUE_SUCCESS)
    {
        /* TODO - Need to handle if job queue is full */
    }
}

/*!
 * @brief This function handles feature event
 */
static void handler_feature_event(uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin_polarity;

#if !defined(MCU_NICLA) /*TODO: Need to check whether this is required for nicla*/
    if (multiio_pin < COINES_SHUTTLE_PIN_MAX && int_pin_usage_native_emulated[multiio_pin])
    {
    	/*lint -e661 */
        stream_feature_line_state[multiio_pin] = 1;
    }
#else
    if (multiio_pin < COINES_SHUTTLE_PIN_MAX)
    {
    	stream_feature_line_state[multiio_pin] = 1;
    }

#endif
}

/*!
 * @brief This function allocate fifo buffers
 */
static void stream_allocate_fifo_buffers(stream_fifo_descriptor_t *fifo_stream_p)
{
    fifo_stream_p->fifo_data = (uint8_t*) stream_memory_alloc(fifo_stream_p->number_bytes * sizeof(uint8_t));

    /* allocate space for the Int data to be read from the sensor */
    if (fifo_stream_p->intline_count)
    {
        fifo_stream_p->DATA_intline = (uint8_t*) stream_memory_alloc(fifo_stream_p->intline_count * sizeof(uint8_t));
    }
}

/*!
 * @brief This function deallocate fifo buffers
 */
static void stream_deallocate_fifo_buffers(stream_fifo_descriptor_t *fifo_stream_p)
{
    stream_safe_free(&fifo_stream_p->fifo_data);
    stream_safe_free(&fifo_stream_p->intline_info);
    stream_safe_free(&fifo_stream_p->DATA_intline);
}

/*!
 * @brief This function allocate buffers
 */
static void stream_allocate_buffers(stream_descriptor_t *stream_p)
{
    /* allocates buffers for reading the data from the sensors - all other structures holding the config data have been
     * allocated when the config was received */
    for (uint8_t chunk_idx = 0; chunk_idx < stream_p->chunk_count; chunk_idx++)
    {
        stream_chunkinfo_t *chunk_p = &stream_p->chunks[chunk_idx];
        if (chunk_p->num_bytes_to_read != 0)
        {
            /* allocate space for the register data to be read from the sensor */
            chunk_p->DATA_chunk = (uint8_t*) stream_memory_alloc(chunk_p->num_bytes_to_read * sizeof(uint8_t));
        }
    }

    /* allocate space for the Int data to be read from the sensor */
    if (stream_p->intline_count)
    {
        stream_p->DATA_intline = (uint8_t*) stream_memory_alloc(stream_p->intline_count * sizeof(uint8_t));
    }
    #ifdef DMA_STREAMING_SUPPORT
    for (uint8_t dma_chunk_idx = 0; dma_chunk_idx < stream_p->dma_chunk_count; dma_chunk_idx++)
    {
        dma_stream_chunkinfo_t *dma_chunk_p = &stream_p->dma_chunks[dma_chunk_idx];
        if (dma_chunk_p->read_page_len != 0)
        {
            /* allocate space for the register data to be read from the sensor */
            dma_chunk_p->data = (uint8_t*) stream_memory_alloc(dma_chunk_p->read_page_len * sizeof(uint8_t));
        }
    }
    #endif /* DMA_STREAMING_SUPPORT */

}

/*!
 * @brief This function deallocate buffers
 */
static void stream_deallocate_buffers(stream_descriptor_t *stream_p)
{
    /* deallocate all buffers (both for sensor data and config data */

    if (stream_p->read_mode > STREAM_READ_N_CHUNKS)
    {
        /* Interrupt status register chunk*/
        stream_chunkinfo_t *chunk_p = &stream_p->chunks[0];

        /* allocate space for the register data to be read from the sensor */
        stream_safe_free(&chunk_p->DATA_chunk);
    }
    else
    {

        for (uint8_t chunk_idx = 0; chunk_idx < stream_p->chunk_count; chunk_idx++)
        {
            stream_chunkinfo_t *chunk_p = &stream_p->chunks[chunk_idx];
            stream_safe_free(&chunk_p->DATA_chunk);
        }
        #ifdef DMA_STREAMING_SUPPORT
        for (uint8_t dma_chunk_idx = 0; dma_chunk_idx < stream_p->dma_chunk_count; dma_chunk_idx++)
        {
            dma_stream_chunkinfo_t *dma_chunk_p = &stream_p->dma_chunks[dma_chunk_idx];
            stream_safe_free(&dma_chunk_p->data);
        }
        #endif /* DMA_STREAMING_SUPPORT */
    }

    stream_safe_free((uint8_t **)&stream_p->chunks); /*lint !e740 */
    stream_safe_free(&stream_p->chunk_mask);
    stream_safe_free(&stream_p->intline_info);
    stream_safe_free(&stream_p->DATA_intline);
    #ifdef DMA_STREAMING_SUPPORT
    stream_safe_free((uint8_t **)&stream_p->dma_chunks); /*lint !e740 */
    #endif /* DMA_STREAMING_SUPPORT */

}

/*!
 * @brief This function set the variable when polling timer expire
 */
static void polling_event_handler()
{
    ++poll_stream_triggered;
}

/*!
 * @brief This function check whether polling/interrupt data is available and send the acquired data
 */
void send_legacy_protocol_streaming_response(void)
{

    if ((stream_settings.stream_mode == STREAM_MODE_POLLING ||
         stream_settings.stream_mode == STREAM_MODE_FIFO_POLLING) && poll_stream_triggered)
    {
        stream_polling_data_acq();
    }
    else if (stream_settings.stream_mode == STREAM_MODE_INTERRUPT)
    {
        (void)job_queue_execute_jobs();
    }
}

/*!
 * @brief This function start streaming
 */
void stream_start(void)
{
    /*TODO: have a common handling of the feature interrupts (set a unique handler for them maybe) */
    stream_descriptor_t *stream_p;
    stream_fifo_descriptor_t *fifo_stream_p;
    uint8_t polling_stream_count = 0;
    enum coines_pin_interrupt_mode interrupt_mode;

    if ((stream_active_count > 0) && (stream_settings.stream_mode == STREAM_MODE_INTERRUPT))
    {
        /* Intialize job_queue and mbuf for interrupt streaming */
        (void)job_queue_init();
        (void)mbuf_init(mbuf_user_evt_handler);
    }
    for (uint8_t i = 0; i < stream_active_count; i++)
    {
        if (stream_settings.stream_mode == STREAM_MODE_FIFO_POLLING)
        {
            fifo_stream_p = &stream_fifo_descriptors;

            /* if GST period is not set, it's impossible to configure a polling stream, so skip it */
            if (stream_settings.GST_period_us == 0)
            {
                continue;
            }

            /* calculate number of GST ticks in this sensor's sampling period */
            if (fifo_stream_p->sampling_period_us <= stream_settings.GST_period_us)
            {
                fifo_stream_p->GST_multiplier = 1;
            }
            else
            {
                fifo_stream_p->GST_multiplier = fifo_stream_p->sampling_period_us / stream_settings.GST_period_us;
            }

            /* initialize GST counter for count-down */
            fifo_stream_p->GST_ticks_counter = fifo_stream_p->GST_multiplier;

            stream_allocate_fifo_buffers(fifo_stream_p);

            ++polling_stream_count;
        }
        else
        {

            stream_p = &stream_descriptors[i];

            stream_allocate_buffers(stream_p);

            if (stream_p->mode == STREAM_MODE_INTERRUPT || stream_p->mode == STREAM_MODE_DMA_INTERRUPT)
            {
                stream_p->data_packet_counter = 0;

                if (stream_p->hw_pin_state == 0)/*active low*/
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
                }
                else
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
                }

                coines_attach_timed_interrupt((enum coines_multi_io_pin)stream_p->data_ready_int,
                                            handler_drdy_int_event,
                                            interrupt_mode);

                /*
                * TODO:
                * Dummy data read since magnetometer interrupt stays high till the sensor data is read.
                */
                if (stream_p->mode == STREAM_MODE_INTERRUPT)
                {
                    if (stream_p->read_mode == STREAM_READ_N_CHUNKS_INTERRUPT_STATUS_FRAMELEN)
                    {
                        if (read_and_send_multiple_fifo_data(stream_p) != 0)
                        {
                            return;
                        }
                    }
                    else if (stream_p->read_mode == STREAM_READ_2_CHUNKS_LITTLEENDIAN_FRAMELEN)
                    {
                        if (read_and_send_single_fifo_data(stream_p) != 0)
                        {
                            return;
                        }
                    }
                    else
                    {
                        if (read_stream_sample(stream_p) != 0)
                        {
                            return;
                        }
                    }
                }
                
            }
            else if ((stream_p->mode == STREAM_MODE_POLLING) || stream_p->mode == STREAM_MODE_DMA_POLLING)
            {
                /* if GST period is not set, it's impossible to configure a polling stream, so skip it */
                if (stream_settings.GST_period_us == 0)
                {
                    continue;
                }

                /* calculate number of GST ticks in this sensor's sampling period */
                if (stream_p->sampling_period_us <= stream_settings.GST_period_us)
                {
                    stream_p->GST_multiplier = 1;
                }
                else
                {
                    stream_p->GST_multiplier = stream_p->sampling_period_us / stream_settings.GST_period_us;
                }

                /* initialize GST counter for count-down */
                stream_p->GST_ticks_counter = stream_p->GST_multiplier;

                if (stream_p->mode == STREAM_MODE_POLLING)
                {
                    if (stream_p->intline_info != NULL)
                    {
                        for (uint8_t int_idx = 0; int_idx < stream_p->intline_count; int_idx++)
                        {
                            if (stream_p->hw_pin_state == 0)/*active low*/
                            {
                                interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
                            }
                            else
                            {
                                interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
                            }

                            coines_attach_interrupt((enum coines_multi_io_pin)stream_p->intline_info[int_idx],
                                                    handler_feature_event,
                                                    interrupt_mode);

                        }
                    }
                }

                ++polling_stream_count;
            }
        }
    }
    if (polling_stream_count > 0)
    {
        /* Timer 0 init for Sensor Polling streaming */
        coines_timer_config(COINES_TIMER_INSTANCE_0, polling_event_handler);
        coines_timer_start(COINES_TIMER_INSTANCE_0, stream_settings.GST_period_us);
    }
}

/*!
 * @brief This API will stop  the streaming
 */
void stream_stop(void)
{
    stream_descriptor_t *stream_p;
    stream_fifo_descriptor_t *fifo_stream_p;
    uint8_t polling_stream_count = 0;
    uint8_t interrupt_stream_count = 0;

    for (uint8_t i = 0; i < stream_active_count; i++)
    {
        if (stream_settings.stream_mode == STREAM_MODE_FIFO_POLLING)
        {
            fifo_stream_p = &stream_fifo_descriptors;
            stream_deallocate_fifo_buffers(fifo_stream_p);
            memset(fifo_stream_p, 0, sizeof(stream_fifo_descriptor_t));
            ++polling_stream_count;
        }
        else
        {
            stream_p = &stream_descriptors[i];

            if ((stream_p->mode == STREAM_MODE_INTERRUPT) || (stream_p->mode == STREAM_MODE_DMA_INTERRUPT))
            {
                bhi_streaming_configured = false;
                coines_detach_timed_interrupt((enum coines_multi_io_pin)stream_p->data_ready_int);
                ++interrupt_stream_count;
            }
            else if (stream_p->mode == STREAM_MODE_POLLING)
            {
                if (stream_p->intline_info != NULL)
                {
                    for (uint8_t int_idx = 0; int_idx < stream_p->intline_count; int_idx++)
                    {
                        coines_detach_interrupt((enum coines_multi_io_pin)stream_p->intline_info[int_idx]);
                    }
                }

                ++polling_stream_count;
            }

            stream_deallocate_buffers(stream_p);

            memset(stream_p, 0, sizeof(stream_descriptor_t));
        }
    }

    stream_active_count = 0;

    if (polling_stream_count > 0)
    {
        /* Timer 0 init for Sensor Polling streaming */
        coines_timer_stop(COINES_TIMER_INSTANCE_0);
    }
    else if(interrupt_stream_count > 0)
    {
        /* Deinit the job_queue and mbuf */
        job_queue_deinit();
        mbuf_deinit();
    }

    /*TODO: delete any pending int */
}

/*!
 * @brief This function will stream the polling data
 */
void stream_polling_data_acq(void)
{
    stream_descriptor_t *stream_p = stream_descriptors;
    stream_fifo_descriptor_t *fifostream_p;
    uint16_t identifier = 0;

    --poll_stream_triggered;
    for (uint8_t i = 0; i < stream_active_count; i++)
    {

        if (stream_settings.stream_mode == STREAM_MODE_POLLING)
        {
            stream_p = &stream_descriptors[i];

            /* decrement each tick down-counter */
            if (stream_p->GST_ticks_counter > 0)
            {
                stream_p->GST_ticks_counter--;
            }

            /* if the tick-counter is 0, it's time to poll the corresponding sensor */
            if (stream_p->GST_ticks_counter == 0)
            {
                /* reload tick-counter */
                stream_p->GST_ticks_counter = stream_p->GST_multiplier;

                if (stream_p->mode == STREAM_MODE_POLLING)
                {
                    if (read_stream_sample(stream_p) == 0)
                    {
                        identifier |= (uint16_t)(1 << i);

                        /* Send the data to communication interface */
                        if (stream_p->read_mode != STREAM_READ_2_CHUNKS_POLLING)
                        {
                            streaming_polling_send_response(stream_p);
                        }
                    }
                }
                #ifdef DMA_STREAMING_SUPPORT
                if (stream_p->mode == STREAM_MODE_DMA_POLLING)
                {
                    if (read_dma_stream_sample(stream_p) == 0)
                    {
                        /* Send the data to communication interface */
                        if (stream_p->read_mode != STREAM_READ_2_CHUNKS_POLLING)
                        {
                            streaming_dma_polling_send_response(stream_p);
                        }
                    }
                    
                }
                #endif /* DMA_STREAMING_SUPPORT */
            }
        }
        else
        {
            /*This feature is implemented for FIFO read feature available in DD2.0 for old sensors
             * In this a timer is started, after the elapse of timer ticks FIFO data is read.*/
            fifostream_p = &stream_fifo_descriptors;
            if (fifostream_p->mode == STREAM_MODE_FIFO_POLLING)
            {
                /* decrement each tick down-counter */
                if (fifostream_p->GST_ticks_counter > 0)
                {
                    fifostream_p->GST_ticks_counter--;
                }

                /* if the tick-counter is 0, it's time to poll the corresponding sensor */
                if (fifostream_p->GST_ticks_counter == 0)
                {
                    /* reload tick-counter */
                    fifostream_p->GST_ticks_counter = fifostream_p->GST_multiplier;

                    if (read_fifo_polling_sample(fifostream_p) == 0)
                    {
                        /* No Multiple channel command is expected*/
                        streaming_fifo_send_response(fifostream_p);

                    }
                }
            }
        }
    }

    /*TODO: Done temporarily for fix for old polling response, need to aligned with the streaming_polling_send_response
     * function */
    if (identifier != 0)
    {
        if (stream_p->read_mode == STREAM_READ_2_CHUNKS_POLLING)
        {
            streaming_polling_old_send_response(identifier);
        } 
    }
    /* Stop streaming, if steaming configured with single sample */
    if (decoder_stream_samples_count == 1)
    {
        decoder_stream_samples_count = 0;
        stream_stop();
    }
}

/*!
 * @brief This function will stream interrupt data
 */
void stream_interrupt_data_acq(uint8_t *p_data)
{
    stream_descriptor_t *stream_p;
    enum coines_pin_value gpio_pin_val = COINES_PIN_VALUE_LOW;
    enum coines_pin_direction gpio_dir;
    /*lint -e826 */
    job_data_t *job_data = (job_data_t*)p_data;

    for (uint8_t i = 0; i < stream_active_count; i++)
    {
        stream_p = &stream_descriptors[i];
        if (stream_p->mode == STREAM_MODE_INTERRUPT)
        {
            /* Find if the pin value is matching */
            /*TODO: use a fast hash-table/mapping */
            if (stream_p->data_ready_int == job_data->multiio_pin)
            {
                if (stream_p->intline_info != NULL)
                {
                    for (uint8_t int_line = 0; int_line < stream_p->intline_count; int_line++)
                    {
                    	coines_get_pin_config((enum coines_multi_io_pin)multi_io_map[stream_p->intline_info[int_line]],
                    			&gpio_dir,
								&gpio_pin_val);
                    	stream_p->DATA_intline[int_line] = (uint8_t) gpio_pin_val;
                    }
                }

                if (stream_p->read_mode == STREAM_READ_N_CHUNKS)
                {
                    if (read_stream_sample(stream_p) == 0)
                    {
                        /* Update timestamp */
                        if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
                        {
                            memcpy(&stream_p->packet_timestamp_us, &job_data->timestamp_us, sizeof(uint64_t));
                        }

                        /* Send the data to communication interface */
                        streaming_interrupt_send_response(stream_p);
                    }
                }
                /* read the configured data chunks for current stream */
                else if (stream_p->read_mode == STREAM_READ_N_CHUNKS_INTERRUPT_STATUS_FRAMELEN)
                {
                    bhi_streaming_configured = true;

                    /* Used in BHY2 fifo streaming. In this response is sent as soon as data is read as huge amount of FIFO
                     * data is read in Multiple packets*/
                    if (read_and_send_multiple_fifo_data(stream_p) != 0)
                    {
                        /* Need to define error scenarios at later stage*/
                    }

                    if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
                    {
                        stream_p->packet_timestamp_us = coines_get_micro_sec();
                    }
                }
                else
                {
                    if (stream_p->read_mode == STREAM_READ_2_CHUNKS_LITTLEENDIAN_FRAMELEN)
                    {
                        /* Used in case of BHY1, where only one FIFO is available*/
                        if (read_and_send_single_fifo_data(stream_p) != 0)
                        {
                            /* Need to define error scenarios at later stage*/
                        }
                    }
                }
            }
        }
        #ifdef DMA_STREAMING_SUPPORT
        if (stream_p->mode == STREAM_MODE_DMA_INTERRUPT)
        {
            /* Find if the pin value is matching */
            /*TODO: use a fast hash-table/mapping */
            if (stream_p->data_ready_int == job_data->multiio_pin)
            {
                if (stream_p->read_mode == STREAM_READ_N_CHUNKS)
                {
                    if (read_dma_stream_sample(stream_p) == 0)
                    {
                        /* Update timestamp */
                        if (stream_settings.ts_mode == STREAM_USE_TIMESTAMP)
                        {
                            memcpy(&stream_p->packet_timestamp_us, &job_data->timestamp_us, sizeof(uint64_t));
                        }

                        /* Send the data to communication interface */
                        streaming_dma_interrupt_send_response(stream_p);
                    }
                }
            }
        }
        #endif /* DMA_STREAMING_SUPPORT */
    }
}

/*!
 * @brief This function will allocates memory
 */
void* stream_memory_alloc(uint32_t size)
{
    void *memptr;

    memptr = malloc(size);
    if (memptr != NULL)
    {
        memset(memptr, 0, size);
    }

    return memptr;
}

/** @}*/
