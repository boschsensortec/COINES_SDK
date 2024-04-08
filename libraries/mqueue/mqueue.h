/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
 */

#ifndef MQUEUE_H
#define MQUEUE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "coines.h"

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif

#define MQUEUE_SUCCESS               0
#define MQUEUE_EMPTY                 -1
#define MQUEUE_FULL                  -2
#define MQUEUE_UNKNOWN_JOB_CALLBACK  -3
#define MQUEUE_INVALID_STATE         -4

#define MQUEUE_DEPTH                 20
#define MQUEUE_DATA_SIZE             50

/**
 * @brief queue state.
 */
typedef enum
{
    MQUEUE_STATE_UNINITIALIZED,
    MQUEUE_STATE_INITIALIZED
} mqueue_state_t;

typedef struct 
{
    uint8_t len;
    uint8_t data[MQUEUE_DATA_SIZE];
}packet_t;

typedef struct 
{
    int8_t front;
    int8_t rear;
    packet_t packet[MQUEUE_DEPTH];
}mqueue_t;

/**
 * @brief This API is used to initialize the mqueue.
 *
 */
int8_t mqueue_init(void);

/**
 * @brief This API is used to add data to the queue.
 *
 *  @param[in] sensor_id : Pointer to data.
 *  @param[in] p_data : Pointer to data.
 *  @param[in] len : Number of bytes to add.
 * 
 *  @retval MQUEUE_SUCCESS -> If data added successfully
 *  @retval MQUEUE_INVALID_STATE -> Mqueue is not initialized
 *  @retval MQUEUE_FULL -> Mqueue is full
 */
int8_t mqueue_add_data(uint8_t sensor_id, uint8_t *p_data, uint8_t len);

/**
 * @brief This API is used to read the data from queue.
 *
 *  @param[in] buff : Pointer to buffer.
 *  @param[in] length : Length of the rsp.
 * 
 *  @retval MQUEUE_SUCCESS -> If data added successfully
 *  @retval MQUEUE_INVALID_STATE -> Mqueue is not initialized
 */
int8_t mqueue_read_rsp(uint8_t *buff, uint16_t *length);

/**
 * @brief This API is used to add data to the queue.
 *
 *  @param[in] sensor_id : Pointer to data.
 *  @param[in] buff : Pointer to buffer.
 *  @param[in] queue_count : Number of streamed data stored in buff.
 * 
 *  @retval MQUEUE_SUCCESS -> If data added successfully
 *  @retval MQUEUE_INVALID_STATE -> Mqueue is not initialized
 */
int8_t mqueue_read_stream_data(uint8_t sensor_id, uint8_t *buff, uint32_t *queue_count);

/**
 * @brief This API is used to de-initialize the mqueue.
 *
 */
void mqueue_deinit(void);

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
}
#endif

#endif /* JOB_MQUEUE_H */