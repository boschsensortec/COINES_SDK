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
 * @file    comm_ringbuffer.c
 * @brief   This module provides supporting data structures for communication interface layer using
 * various interfaces (USB,VCOM,BLE)
 *
 */

/*!
 * @defgroup comm_intf_api comm_intf
 * @{*/

/***********************************************************************************/
/* own header files */
/**********************************************************************************/
#include "comm_ringbuffer.h"
#include "coines.h"
#include "coines_defs.h"
#include "stdlib.h"

static const uint8_t packet_delimiter[4] = { 0x22, 0x06, 0x19, 0x93 };

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/*!
 * @brief This API is used for initializing circular buffer
 *
 * @param[in] size: size of the buffer
 *
 * @return pointer to the new ringbuffer structure if successful, else a NULL value
 */
comm_ringbuffer_t* comm_ringbuffer_create(uint32_t size)
{
    if (size == 0)
    {
        return NULL;
    }

    /* allocate handler structure and buffer for the ringbuffer */
    comm_ringbuffer_t* rbuf = (comm_ringbuffer_t*)malloc(sizeof(comm_ringbuffer_t));
    uint8_t* buff_ptr = (uint8_t *)malloc(size);

    /* if memory allocation was successful, configure the handler */
    if (rbuf && buff_ptr)
    {
        rbuf->Base = buff_ptr;
        rbuf->Rptr = buff_ptr;
        rbuf->Wptr = buff_ptr;
        rbuf->Count = 0;
        rbuf->packetCounter = 0;
        rbuf->Size = size;
    }
    else
    {
        /* memory allocation failed, clean up any remains */
        comm_ringbuffer_delete(rbuf);

        return NULL;
    }

    return rbuf;
}

/*!
 * @brief This API is used for deleting circular buffer
 *
 * @param[out] rbuf: Pointer to the circular buffer data structure
 *
 * @return void
 */
void comm_ringbuffer_delete(comm_ringbuffer_t* rbuf)
{
    if (rbuf)
    {
        if (rbuf->Base)
        {
            free(rbuf->Base);
            rbuf->Base = NULL;
        }

        free(rbuf);
    }
}

/**
 *  @brief Writes a variable amount of data to the ringbuffer
 *
 *  @param[out] cirbuf : Pointer to the circular buffer data structure *
 *  @param[in] buffer : The byte that is to be written into
 *  @param[in] write_len : length param
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_write(comm_ringbuffer_t* rbuf, const uint8_t *buffer, uint32_t write_len)
{
    if ((rbuf != NULL) && (buffer != NULL))
    {
        /* check if there is enough space in the ringbuffer */
        if (write_len > rbuf->Size - rbuf->Count)
        {
            return COINES_E_FAILURE;
        }

        while (write_len-- != 0)
        {
            *rbuf->Wptr++ = *buffer++;
            rbuf->Count++;

            if ((rbuf)->Wptr == (rbuf)->Base + (rbuf)->Size)
            {
                (rbuf)->Wptr = (rbuf)->Base;
            }
        }

        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

/**
 *  @brief Writes a variable amount of data to the ringbuffer as a complete package, with delimiter and incrementing of pkg_counter
 *
 *  @param[out] rbuf : Pointer to the circular buffer data structure *
 *  @param[in] buffer : The byte that is to be written into
 *  @param[in] write_len : length param
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_write_packet(comm_ringbuffer_t* rbuf, const uint8_t *buffer, uint32_t write_len)
{
    if ((rbuf != NULL) && (buffer != NULL))
    {

        int8_t ret_val = COINES_SUCCESS;

        ret_val += comm_ringbuffer_write(rbuf, buffer, write_len);
        ret_val += comm_ringbuffer_add_delimiter(rbuf);

        if (ret_val == COINES_SUCCESS)
        {
            rbuf->packetCounter++;
        }

        return ret_val;
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

/**
 *  @brief Quickly return (pop) a byte from the ring buffer
 *
 *  @param[in] rbuf : Pointer to the ring buffer data structure
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_pop(comm_ringbuffer_t* rbuf, uint8_t *bt)
{
    if ((rbuf != NULL) && (bt != NULL))
    {

        /* check if there is enough data in the ringbuffer */
        if (rbuf->Count == 0)
        {
            return COINES_E_FAILURE;
        }

        *bt = *rbuf->Rptr++;
        rbuf->Count--;

        if ((rbuf)->Rptr == (rbuf)->Base + (rbuf)->Size)
        {
            (rbuf)->Rptr = (rbuf)->Base;
        }

        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

/**
 *  @brief Read a data packet from the circular buffer
 *
 *  @param[in] cirbuf : Pointer to the circular buffer data structure
 *  @param[out] buffer : Must point to a uint8_t where the API could store the bytes fetched from the ring buffer
 *  @param[in] packet_count : packet count
 *
 *  @return Result of API execution status
 */
uint32_t comm_ringbuffer_read(comm_ringbuffer_t * rbuf, uint8_t *buffer, uint32_t packet_count)
{
    int rslt;
    uint32_t idx_bt = 0, payload_bytes_read = 0;
    uint8_t idx_packet, continue_read;

    /* packet_delimiter constant string is the last string of any given packet. The below logic will keep reading
     * data from the ring buffer until COMM_INTF_CIRC_BUFF_DELIMITER sequence is read.
     * There can be more packets in the buffer, and the required packet count will be read, if available in the buffer */

    for (idx_packet = 0; idx_packet < packet_count; idx_packet++)
    {
        /*TODO: DEBUG printf("rbuf packet %d   idx %d\n", idx_packet, idx_bt); */

        /* read first 3 bytes - the packet cannot be shorter than 4 bytes, as the separator has 4 bytes */
        rslt = comm_ringbuffer_pop(rbuf, &buffer[idx_bt++]);
        rslt += comm_ringbuffer_pop(rbuf, &buffer[idx_bt++]);
        rslt += comm_ringbuffer_pop(rbuf, &buffer[idx_bt++]);

        /* if there was an error exit with the previously read data, else update the counter of useful data */
        if (rslt != COINES_SUCCESS)
        {
            return payload_bytes_read;
        }
        else
        {
            payload_bytes_read += 3;
        }

        continue_read = 1;
        while (continue_read) /*TODO: until end of stream */
        {
            /* read next byte */
            rslt = comm_ringbuffer_pop(rbuf, &buffer[idx_bt++]);

            /* if there was an error reading the data or if end-of-stream, then exit, else update the counter of useful
             * data */
            if (rslt != COINES_SUCCESS)
            {
                return payload_bytes_read;
            }
            else
            {
                ++payload_bytes_read;
            }

            /* check a match for the delimiter - start from last character */
            if (buffer[idx_bt - 1] == packet_delimiter[3])
            {
                if ((buffer[idx_bt - 2] == packet_delimiter[2]) && (buffer[idx_bt - 3] == packet_delimiter[1]) &&
                    (buffer[idx_bt - 4] == packet_delimiter[0]))
                {
                    /* delimiter match, remove it from buffer and read byte count */
                    idx_bt -= 4;
                    payload_bytes_read -= 4;
                    rbuf->packetCounter--;
                    continue_read = 0;
                    DEBUG_PRINT("TERMINATOR MATCH Packet: %d   Buffer byte index: %d\n", idx_packet, idx_bt);
                }
            }
        }
    }

    return payload_bytes_read;
}

/**
 *  @brief Reset all state variables and content from a ringbuffer
 *
 *  @param[in] cirbuf : Pointer to the circular buffer data structure
 *
 *  @return void
 */
void comm_ringbuffer_reset(comm_ringbuffer_t * rbuf)
{
    if (rbuf != NULL)
    {
        rbuf->Rptr = rbuf->Base;
        rbuf->Wptr = rbuf->Base;
        rbuf->Count = 0;
        rbuf->packetCounter = 0;
    }
}

/**
 *  @brief This API adds delimiter to the ring buffer
 *
 *  @param[in] rbuf : Pointer to the circular buffer data structure
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_add_delimiter(comm_ringbuffer_t * rbuf)
{
    return comm_ringbuffer_write(rbuf, packet_delimiter, 4);
}
