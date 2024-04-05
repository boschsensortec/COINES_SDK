/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
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
 * @file    mbuf.h
 * @date    02 Dec 2022
 * @brief   Header file to store data in mbuf
 *
 */

#ifndef _MBUF_H_
#define _MBUF_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

/* Start of CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#define MBUF_DEPTH            2
#define MBUF_DATA_SIZE        1024

#define MBUF_SUCCESS          0
#define MBUF_E_INIT_FAIL      -1
#define MBUF_E_READ_FAIL      -2
#define MBUF_E_INVALID_STATE  -3

/**
 * @brief Mbuf state.
 */
typedef enum
{
    MBUF_STATE_UNINITIALIZED,
    MBUF_STATE_INITIALIZED
} mbuf_state_t;

/**
 * @brief Mbuf events.
 */
typedef enum {
    MBUF_EVT_BUFFER_FULL
} mbuf_evt_type_t;

/**
 * @brief Mbuf event handler.
 *
 * @param[in] event  Event.
 */
typedef void (*mbuf_evt_handler_t)(mbuf_evt_type_t event);

/**
 * @brief Mbuf configuration structure.
 */
typedef struct
{
    uint8_t data[MBUF_DEPTH][MBUF_DATA_SIZE];
    uint16_t idx[MBUF_DEPTH];
    mbuf_evt_handler_t handler;
    mbuf_state_t state;
} mbuf_t;

/**
 * @brief This API is used to initialize the Mbuf.
 *
 *  @param[in] event_handler : Event handler provided by the user.
 *
 *
 *  @retval MBUF_SUCCESS -> If initialization was successful or already intialized
 */
int8_t mbuf_init(mbuf_evt_handler_t event_handler);

/**
 * @brief This API is used to add data to the mbuf.
 *
 *  @param[in] p_data : Pointer to data.
 *  @param[in] length : Number of bytes to add.
 *
 */
void mbuf_add_to_buffer(uint8_t *p_data, uint16_t length);

/**
 * @brief This API is used to get the data from the mbuf.
 *
 *  @param[in] p_address: Pointer address to get the buffer.
 *  @param[in] p_length : Pointer to get number of bytes available in buffer.
 *
 *  @retval MBUF_SUCCESS -> If data available in the buffer
 *  @retval MBUF_E_READ_FAIL -> If data is not available in the buffer
 */
int8_t mbuf_get_from_buffer(uint8_t **p_address, uint16_t *p_length);

/**
 * @brief This API is used to update the buffer status.
 *
 */
void mbuf_update_processed_buffer();

/**
 * @brief This API is used to get the active buffer length.
 *
 *  @retval Return active buffer length
 */
uint16_t mbuf_get_buffer_length();

/*!
 * @brief This API is used to deinit the mbuf.
 *
 */
void mbuf_deinit(void);

/* End of CPP guard */
#ifdef __cplusplus
}
#endif

#endif /* _MULTI_BUFFERING_H_ */
