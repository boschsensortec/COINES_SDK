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
 */
#include "mqueue.h"
#include "coines.h"

static mqueue_t mqueue[COINES_MAX_SENSOR_COUNT];

static volatile mqueue_state_t mqueue_state = MQUEUE_STATE_UNINITIALIZED;
static volatile uint8_t read_queue_idx;

static void queue_update_executed_idx();
static void reset_queue();
static bool is_queue_full(mqueue_t *mqueue);

static void copy_sensor_sample(mqueue_t *p_mqueue, uint8_t *buff, uint8_t *sensor_sample, uint8_t len);

/**
 * @brief This API is used to intialize the queue.
 *
 */
int8_t mqueue_init(void)
{
    if (mqueue_state == MQUEUE_STATE_UNINITIALIZED)
    {
        for (uint8_t q_idx = 0; q_idx < COINES_MAX_SENSOR_COUNT; q_idx++)
        {
            mqueue[q_idx].front = MQUEUE_EMPTY;
            mqueue[q_idx].rear = MQUEUE_EMPTY;
            for (uint8_t j = 0; j < MQUEUE_DEPTH; j++)
            {
                memset(mqueue[q_idx].packet[j].data, 0, MQUEUE_DATA_SIZE);
            }
        }

        mqueue_state = MQUEUE_STATE_INITIALIZED;
    }

    return MQUEUE_SUCCESS;
}

/**
 * @brief This API is used to reset the queue.
 *
 */
static void reset_queue()
{
    mqueue[read_queue_idx].rear = MQUEUE_EMPTY;
    mqueue[read_queue_idx].front = MQUEUE_EMPTY;
}

static bool is_queue_full(mqueue_t *mqueue)
{
    if ((mqueue->front == mqueue->rear + 1) || (mqueue->front == 0 && (mqueue->rear == MQUEUE_DEPTH - 1)))
    {
        return true;
    }

    return false;
}

/**
 * @brief This API is used to add data to the queue.
 *
 */
int8_t mqueue_add_data(uint8_t sensor_id, uint8_t *p_data, uint8_t len)
{
    mqueue_t *p_mqueue;

    if (mqueue_state != MQUEUE_STATE_UNINITIALIZED)
    {
        p_mqueue = &mqueue[sensor_id];
        if (is_queue_full(p_mqueue))
        {
            return MQUEUE_FULL;
        }
        else
        {
            if (p_mqueue->front == MQUEUE_EMPTY)
            {
                p_mqueue->front = 0;
            }

            p_mqueue->rear = (p_mqueue->rear + 1) % MQUEUE_DEPTH;

            if (p_data != NULL)
            {
                p_mqueue->packet[p_mqueue->rear].len = len;
                memcpy(p_mqueue->packet[p_mqueue->rear].data, p_data, len);
            }
        }

        return MQUEUE_SUCCESS;
    }
    else
    {
        return MQUEUE_INVALID_STATE;
    }
}

/**
 * @brief This API is used to read the data from queue.
 *
 */
int8_t mqueue_read_rsp(uint8_t *buff, uint16_t *length)
{
    mqueue_t *p_mqueue = &mqueue[0];

    if (mqueue_state != MQUEUE_STATE_UNINITIALIZED)
    {
        if (p_mqueue->front == MQUEUE_EMPTY)
        {
            return MQUEUE_EMPTY;
        }
        else
        {

            memcpy(buff, p_mqueue->packet[p_mqueue->front].data, p_mqueue->packet[p_mqueue->front].len);
            *length = p_mqueue->packet[p_mqueue->front].len;
            if (p_mqueue->front == p_mqueue->rear)
            {
                coines_execute_critical_region(reset_queue);
            }
            else
            {
                coines_execute_critical_region(queue_update_executed_idx);
            }

            return MQUEUE_SUCCESS;
        }
    }
    else
    {
        return MQUEUE_INVALID_STATE;
    }
}

/**
 * @brief This API is used to copy the sensor sample.
 *
 */
static void copy_sensor_sample(mqueue_t *p_mqueue, uint8_t *buff, uint8_t *sensor_sample, uint8_t len)
{
    memcpy(buff, sensor_sample, len);
    if (p_mqueue->front == p_mqueue->rear)
    {
        coines_execute_critical_region(reset_queue);
    }
    else
    {
        coines_execute_critical_region(queue_update_executed_idx);
    }
}

/**
 * @brief This API is used to read the streamed data from queue.
 *
 */
int8_t mqueue_read_stream_data(uint8_t sensor_id, uint8_t *buff, uint32_t *queue_count)
{
    uint8_t idx = 0;
    mqueue_t *p_mqueue = &mqueue[sensor_id];
    uint32_t write_idx = 0;

    read_queue_idx = sensor_id;
    uint8_t samples_count = 0;
    int8_t current_count = p_mqueue->rear;

    if (mqueue_state != MQUEUE_STATE_UNINITIALIZED)
    {
        if (p_mqueue->front == MQUEUE_EMPTY)
        {
            *queue_count = 0;

            return MQUEUE_EMPTY;
        }
        else
        {
            for (idx = p_mqueue->front; idx != current_count; idx = (idx + 1) % MQUEUE_DEPTH)
            {
                ++samples_count;

                /* Loop runs till N-1(queue_count) */
                copy_sensor_sample(p_mqueue, (buff + write_idx), p_mqueue->packet[idx].data, p_mqueue->packet[idx].len);
                write_idx += p_mqueue->packet[idx].len;
            }

            /* Read Nth element from queue */
            ++samples_count;
            copy_sensor_sample(p_mqueue, (buff + write_idx), p_mqueue->packet[idx].data, p_mqueue->packet[idx].len);
            write_idx += p_mqueue->packet[idx].len;

            *queue_count = samples_count;

            return MQUEUE_SUCCESS;
        }
    }
    else
    {
        return MQUEUE_INVALID_STATE;
    }
}

/**
 * @brief This API is used to update the job executed status.
 *
 */
static void queue_update_executed_idx()
{
    mqueue[read_queue_idx].front = (mqueue[read_queue_idx].front + 1) % MQUEUE_DEPTH;
}

/**
 * @brief This API is used to deinit the job_queue.
 *
 */
void mqueue_deinit(void)
{
    if (mqueue_state != MQUEUE_STATE_UNINITIALIZED)
    {
        coines_execute_critical_region(reset_queue);
    }
}
