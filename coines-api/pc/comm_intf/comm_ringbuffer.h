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
 * @file    comm_intf.h
 * @brief This module provides supporting data structures for communication interface layer using
 * various interfaces (USB,VCOM,BLE)
 *
 */

/*!
 * @addtogroup comm_intf_api
 * @{*/

#ifndef COMM_INTF_COMM_RINGBUFFER_H_
#define COMM_INTF_COMM_RINGBUFFER_H_

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>

/*!
 * * @brief structure used to hold the circular buffer information.
 */
typedef struct
{
    uint8_t * Base; /**< Pointer to the base of the user-supplied buffer */
    uint8_t * Wptr; /**< Write pointer. NOT to be changed by hand */
    const uint8_t * Rptr; /**< Read pointer. NOT to be changed by hand */
    uint32_t Count; /**< Number of unread bytes currently in the buffer. May be read */
    uint32_t Size; /**< Maximum number of bytes in the user-supplied buffer. Must be set during init */
    uint32_t packetCounter; /**< Packet counter */
} comm_ringbuffer_t;

/*!
 * @brief This API is used for initializing circular buffer
 *
 * @param[out] rbuf: Pointer to the circular buffer data structure
 *
 * @return void
 */
comm_ringbuffer_t* comm_ringbuffer_create(uint32_t size);

/*!
 * @brief This API is used for deleting circular buffer
 *
 * @param[out] rbuf: Pointer to the circular buffer data structure
 *
 * @return void
 */
void comm_ringbuffer_delete(comm_ringbuffer_t* rbuf);

/**
 *  @brief This API adds delimiter to the ring buffer
 *
 *  @param[in] rbuf : Pointer to the circular buffer data structure
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_add_delimiter(comm_ringbuffer_t * rbuf);

/**
 *  @brief Writes a variable amount of data to the ringbuffer
 *
 *  @param[out] cirbuf : Pointer to the circular buffer data structure *
 *  @param[in] buffer : The byte that is to be written into
 *  @param[in] write_len : length param
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_write(comm_ringbuffer_t * rbuf, const uint8_t *buffer, uint32_t write_len);

/**
 *  @brief Writes a variable amount of data to the ringbuffer as a complete package, with delimiter and incrementing of pkg_counter
 *
 *  @param[out] rbuf : Pointer to the circular buffer data structure *
 *  @param[in] buffer : The byte that is to be written into
 *  @param[in] write_len : length param
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_write_packet(comm_ringbuffer_t * rbuf, const uint8_t *buffer, uint32_t write_len);

/**
 *  @brief Quickly return (pop) a byte from the ring buffer
 *
 *  @param[in] rbuf : Pointer to the ring buffer data structure
 *
 *  @return Result of API execution status
 */
int8_t comm_ringbuffer_pop(comm_ringbuffer_t* rbuf, uint8_t *bt);

/**
 *  @brief Read a data packet from the circular buffer
 *
 *  @param[in] cirbuf : Pointer to the circular buffer data structure
 *  @param[out] buffer : Must point to a uint8_t where the API could store the bytes fetched from the ring buffer
 *  @param[in] packet_count : packet count
 *
 *  @return Result of API execution status
 */
uint32_t comm_ringbuffer_read(comm_ringbuffer_t * rbuf, uint8_t *buffer, uint32_t packet_count);

/**
 *  @brief Reset all state variables and content from a ringbuffer
 *
 *  @param[in] cirbuf : Pointer to the circular buffer data structure
 *
 *  @return void
 */
void comm_ringbuffer_reset(comm_ringbuffer_t * rbuf);

#endif /* COMM_INTF_COMM_RINGBUFFER_H_ */
