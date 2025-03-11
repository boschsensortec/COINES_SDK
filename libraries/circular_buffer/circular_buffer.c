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

/*********************************************************************/
/* other header files */
/**********************************************************************/
#include <stdio.h>
#include "circular_buffer.h"
#include "coines.h"

/*********************************************************************/
/* static variables */
/*********************************************************************/

/*********************************************************************/
/* functions */
/*********************************************************************/

/**
 * @brief This API is used to reset the circular buffer.
 *
 */
void circular_buffer_reset(circular_buffer_t *circular_buf)
{
    circular_buf->head = 0;
    circular_buf->tail = 0;
    circular_buf->full = false;
    memset(circular_buf->buffer, 0, circular_buf->max);
}

/*!
 * @brief This API is used to do circular buffer initialization
 *
 */
int8_t circular_buffer_init(circular_buffer_t *circular_buf, uint32_t size)
{
    circular_buf->buffer = (uint8_t *)malloc(size);
    if (!circular_buf->buffer)
    {
        return CIRCULAR_BUFFER_ERROR_MALLOC;
    }

    circular_buf->max = size;
    circular_buffer_reset(circular_buf);

    if (pthread_mutex_init(&circular_buf->mutex, NULL) != 0)
    {
        free((void*)circular_buf->buffer);
        circular_buf->buffer = NULL;

        return CIRCULAR_BUFFER_ERROR_MUTEX;
    }

    return CIRCULAR_BUFFER_SUCCESS;
}

/*!
 * @brief This API is used to free the circular buffer resources
 *
 */
void circular_buffer_free(circular_buffer_t *circular_buf)
{
    if (!circular_buf->buffer)
    {
        return;
    }

    /* Destroy mutex */
    pthread_mutex_destroy(&circular_buf->mutex);

    free((void*)circular_buf->buffer);
    circular_buf->buffer = NULL;
    circular_buf->max = 0;
    circular_buf->head = 0;
    circular_buf->tail = 0;
    circular_buf->full = false;
}

/*!
 * @brief This API is used to put data into the circular buffer
 */
int8_t circular_buffer_put(circular_buffer_t *circular_buf, uint8_t *data, uint16_t len)
{
    if (!circular_buf->buffer || !data)
    {
        return CIRCULAR_BUFFER_ERROR_NULL;
    }

    pthread_mutex_lock(&circular_buf->mutex);

    if (circular_buffer_is_full(circular_buf))
    {
        pthread_mutex_unlock(&circular_buf->mutex);

        return CIRCULAR_BUFFER_ERROR_FULL;
    }

    size_t space_left = circular_buf->max - circular_buf->head;
    if (len <= space_left)
    {
        memcpy(circular_buf->buffer + circular_buf->head, data, len);
        circular_buf->head = (circular_buf->head + len) % circular_buf->max;
    }
    else
    {
        memcpy(circular_buf->buffer + circular_buf->head, data, space_left);
        memcpy(circular_buf->buffer, data + space_left, len - space_left);
        circular_buf->head = len - space_left;
    }

    circular_buf->full = (circular_buf->head == circular_buf->tail);

    pthread_mutex_unlock(&circular_buf->mutex); /* Release the mutex */
    return CIRCULAR_BUFFER_SUCCESS;
}

/*!
 * @brief This API is used to get data from the circular buffer
 */
int8_t circular_buffer_get(circular_buffer_t *circular_buf, uint8_t *data, uint16_t len)
{
    if (!circular_buf->buffer || !data)
    {
        return CIRCULAR_BUFFER_ERROR_NULL;
    }

    pthread_mutex_lock(&circular_buf->mutex);

    if (circular_buffer_size(circular_buf) < len)
    {
        pthread_mutex_unlock(&circular_buf->mutex);

        return CIRCULAR_BUFFER_ERROR_EMPTY;
    }

    size_t space_left = circular_buf->max - circular_buf->tail;
    if (len <= space_left)
    {
        memcpy(data, circular_buf->buffer + circular_buf->tail, len);
        circular_buf->tail = (circular_buf->tail + len) % circular_buf->max;
    }
    else
    {
        memcpy(data, circular_buf->buffer + circular_buf->tail, space_left);
        memcpy(data + space_left, circular_buf->buffer, len - space_left);
        circular_buf->tail = len - space_left;
    }

    circular_buf->full = false;

    pthread_mutex_unlock(&circular_buf->mutex); /* Release the mutex */
    return CIRCULAR_BUFFER_SUCCESS;
}

/*!
 * @brief This API is used to check if the circular buffer is empty
 */
bool circular_buffer_is_empty(circular_buffer_t *circular_buf)
{
    return !circular_buf->full && (circular_buf->head == circular_buf->tail);
}

/*!
 * @brief This API is used to check if the circular buffer is full
 */
bool circular_buffer_is_full(circular_buffer_t *circular_buf)
{
    return circular_buf->full;
}

/**
 * @brief This API is used to calculates the number of elements currently in the circular buffer.
 *
 */
uint32_t circular_buffer_size(circular_buffer_t *circular_buf)
{
    uint32_t size = circular_buf->max;

    if (!circular_buf->full)
    {
        if (circular_buf->head >= circular_buf->tail)
        {
            size = (circular_buf->head - circular_buf->tail);
        }
        else
        {
            size = (circular_buf->max + circular_buf->head) - circular_buf->tail;
        }
    }

    return size;
}

/**
 * @brief This API is used to calculates the amount of free space available in the circular buffer.
 *
 */
uint32_t circular_buffer_free_space(circular_buffer_t *circular_buf)
{
    uint32_t size = circular_buffer_size(circular_buf);
    uint32_t free_space = circular_buf->max - size;

    if (circular_buf->full)
    {
        free_space = 0;
    }

    return free_space;
}
