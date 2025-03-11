/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/* system header files */
/**********************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

/*********************************************************************/
/* macro definitions */
/**********************************************************************/
#define CIRCULAR_BUFFER_SUCCESS       0
#define CIRCULAR_BUFFER_ERROR_NULL    -1
#define CIRCULAR_BUFFER_ERROR_FULL    -2
#define CIRCULAR_BUFFER_ERROR_EMPTY   -3
#define CIRCULAR_BUFFER_ERROR_MALLOC  -4
#define CIRCULAR_BUFFER_ERROR_MUTEX   -5

/*********************************************************************/
/* typedef definitions */
/**********************************************************************/
typedef struct
{
    uint8_t *buffer;
    uint32_t head;
    uint32_t tail;
    uint32_t max;
    bool full;
    pthread_mutex_t mutex; /* Mutex for thread safety */
} circular_buffer_t;

/*********************************************************************/
/* function prototype declarations */
/**********************************************************************/

/*!
 * @brief This API is used to do circular buffer initialization
 *
 */
int8_t circular_buffer_init(circular_buffer_t *circular_buf, uint32_t size);

/*!
 * @brief This API is used to free the circular buffer resources
 *
 */
void circular_buffer_free(circular_buffer_t *circular_buf);

/*!
 * @brief This API is used to put data into the circular buffer
 *
 * @param data Pointer to the data to be put into the buffer
 * @param len Length of the data
 * @return int8_t Returns CIRCULAR_BUFFER_SUCCESS if successful, otherwise an error code
 */
int8_t circular_buffer_put(circular_buffer_t *circular_buf, uint8_t *data, uint16_t len);

/*!
 * @brief This API is used to get data from the circular buffer
 *
 * @param data Pointer to store the retrieved data
 * @param len Length of the data to be retrieved
 * @return int8_t Returns CIRCULAR_BUFFER_SUCCESS if successful, otherwise an error code
 */
int8_t circular_buffer_get(circular_buffer_t *circular_buf, uint8_t *data, uint16_t len);

/*!
 * @brief This API is used to check if the circular buffer is empty
 *
 * @return bool Returns true if the buffer is empty, false otherwise
 */
bool circular_buffer_is_empty(circular_buffer_t *circular_buf);

/*!
 * @brief This API is used to check if the circular buffer is full
 *
 * @return bool Returns true if the buffer is full, false otherwise
 */
bool circular_buffer_is_full(circular_buffer_t *circular_buf);

/**
 * @brief This API is used to calculates the number of elements currently in the circular buffer.
 *
 * @return uint32_t Return number of elements currently in the circular buffer.
 */
uint32_t circular_buffer_size(circular_buffer_t *circular_buf);

/**
 * @brief This API is used to calculates the amount of free space available in the circular buffer.
 *
 * @return uint32_t Return amount of free space available in the circular buffer.
 */
uint32_t circular_buffer_free_space(circular_buffer_t *circular_buf);

/**
 * @brief This API is used to reset the circular buffer.
 *
 */
void circular_buffer_reset(circular_buffer_t *circular_buf);

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
}
#endif

#endif /* CIRCULAR_BUFFER_H */
