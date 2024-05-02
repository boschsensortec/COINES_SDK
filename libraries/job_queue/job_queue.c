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

#include "job_queue.h"
#include "coines.h"

typedef struct
{
    job_callback callback;
    bool ready_to_process;
    uint8_t data[JOB_QUEUE_DATA_SIZE]; /* Used to store application data */
} job_queue_t;

job_queue_t job_queue[JOB_QUEUE_DEPTH];
static volatile int8_t job_queue_idx = JOB_QUEUE_EMPTY;
static volatile int8_t exec_callback_idx = JOB_QUEUE_EMPTY;
static volatile int8_t curr_job_queue_idx = JOB_QUEUE_EMPTY;
static volatile job_queue_state_t job_queue_state = JOB_QUEUE_STATE_UNINITIALIZED;
static void job_queue_update_job_executed_idx();
static bool is_executing_job = false;

static void reset_job_queue(void);
static bool is_job_queue_full(void);
static void update_curr_queue_count(void);

/**
 * @brief This API is used to intialize the job queue.
 *
 */
int8_t job_queue_init(void)
{
    if (job_queue_state == JOB_QUEUE_STATE_UNINITIALIZED)
    {
        for (uint8_t i = 0; i < JOB_QUEUE_DEPTH; i++)
        {
            job_queue[i].callback = NULL;
            job_queue[i].ready_to_process = false;
            memset(&job_queue[i].data, 0, JOB_QUEUE_DATA_SIZE);
        }

        job_queue_state = JOB_QUEUE_STATE_INITIALIZED;
    }

    return JOB_QUEUE_SUCCESS;
}

/**
 * @brief This API is used to reset the queue.
 *
 */
static void reset_job_queue(void)
{
    job_queue_idx = JOB_QUEUE_EMPTY;
    exec_callback_idx = JOB_QUEUE_EMPTY;
}

static bool is_job_queue_full()
{
    if ((exec_callback_idx == job_queue_idx + 1) || (exec_callback_idx == 0 && (job_queue_idx == JOB_QUEUE_DEPTH - 1)))
    {
        return true;
    }

    return false;
}

/**
 * @brief This API is used to add job to the queue.
 *
 */
int8_t job_queue_add_job(job_callback callback, uint8_t *p_data, bool ready_to_process)
{
    if (job_queue_state != JOB_QUEUE_STATE_UNINITIALIZED)
    {
        if (is_job_queue_full())
        {
            return JOB_QUEUE_FULL;
        }
        else
        {
            if (exec_callback_idx == JOB_QUEUE_EMPTY)
            {
                exec_callback_idx = 0;
            }

            job_queue_idx = (job_queue_idx + 1) % JOB_QUEUE_DEPTH;
            job_queue[job_queue_idx].callback = callback;
            job_queue[job_queue_idx].ready_to_process = ready_to_process;
            if (p_data != NULL)
            {
                memcpy(&job_queue[job_queue_idx].data, p_data, JOB_QUEUE_DATA_SIZE);
            }
        }

        return JOB_QUEUE_SUCCESS;
    }
    else
    {
        return JOB_QUEUE_INVALID_STATE;
    }
}

/**
 * @brief This API is used to change status of the job.
 *
 */
int8_t job_queue_ready_to_run_job(job_callback callback)
{
    if (job_queue_state != JOB_QUEUE_STATE_UNINITIALIZED)
    {
        for (uint8_t i = 0; i <= job_queue_idx; i++)
        {
            if (job_queue[i].callback == callback)
            {
                job_queue[i].ready_to_process = true;

                return JOB_QUEUE_SUCCESS;
            }
        }

        return JOB_QUEUE_UNKNOWN_JOB_CALLBACK;
    }
    else
    {
        return JOB_QUEUE_INVALID_STATE;
    }
}

/**
 * @brief This API is used to execute the job.
 *
 */
static void execute_job(job_queue_t *job)
{
    if (job->callback && (job->ready_to_process == true))
    {
        job->callback(job->data);
        job->ready_to_process = false;
        job->callback = NULL;
        coines_execute_critical_region(job_queue_update_job_executed_idx);
    }
}

/**
 * @brief This API is used to execute the jobs in job queue.
 *
 */
int8_t job_queue_execute_jobs(void)
{
    int8_t idx;
    
    if ((job_queue_state != JOB_QUEUE_STATE_UNINITIALIZED) && (!is_executing_job))
    {
        if (exec_callback_idx == JOB_QUEUE_EMPTY)
        {
            return JOB_QUEUE_EMPTY;
        }
        else{
            is_executing_job = true;
            coines_execute_critical_region(update_curr_queue_count);
            for (idx = exec_callback_idx; idx != curr_job_queue_idx; idx = (idx + 1) % JOB_QUEUE_DEPTH)
            {
                /* Loop runs till N-1(queue_count) */
                execute_job(&job_queue[idx]);
            }
            /* Execute Nth job in job_queue */
            execute_job(&job_queue[idx]);
            is_executing_job = false;

            return JOB_QUEUE_SUCCESS;
        }  
    }
    else
    {
        return JOB_QUEUE_INVALID_STATE;
    }
}

/**
 * @brief This API is used to update the job executed status.
 *
 */
static void job_queue_update_job_executed_idx()
{
    if (exec_callback_idx == job_queue_idx)
    {
        reset_job_queue();
    }
    else
    {
        exec_callback_idx = (exec_callback_idx + 1) % JOB_QUEUE_DEPTH;
    }
}

/**
 * @brief This API is used to deinit the job_queue.
 *
 */
void job_queue_deinit(void)
{
    if(job_queue_state != JOB_QUEUE_STATE_UNINITIALIZED)
    {
        coines_execute_critical_region(reset_job_queue);
        job_queue_state = JOB_QUEUE_STATE_UNINITIALIZED;
    }
}

/**
 * @brief This API is used to update current queue count.
 *
 */
static void update_curr_queue_count(void)
{
    curr_job_queue_idx = job_queue_idx; 
}