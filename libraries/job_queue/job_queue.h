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

#ifndef JOB_QUEUE_H
#define JOB_QUEUE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif

#define JOB_QUEUE_SUCCESS               0
#define JOB_QUEUE_EMPTY                 -1
#define JOB_QUEUE_FULL                  -2
#define JOB_QUEUE_UNKNOWN_JOB_CALLBACK  -3
#define JOB_QUEUE_INVALID_STATE         -4

#define JOB_QUEUE_DEPTH                 20
#define JOB_QUEUE_DATA_SIZE             20

/**
 * @brief Job queue state.
 */
typedef enum
{
    JOB_QUEUE_STATE_UNINITIALIZED,
    JOB_QUEUE_STATE_INITIALIZED
} job_queue_state_t;

/**
 * @brief Job queue callback prototype.
 *
 * @param p_data   Pointer to data.
 *
 */
typedef void (*job_callback)(uint8_t *p_data);

/**
 * @brief This API is used to intialize the job queue.
 *
 *
 *  @retval JOB_QUEUE_SUCCESS -> If initialization was successful or already intialized
 */
int8_t job_queue_init(void);

/**
 * @brief This API is used to add job to the queue.
 *
 *  @param[in] callback : Function provided by the user.
 *  @param[in] p_data : Pointer to application data.
 *
 *  @retval JOB_QUEUE_SUCCESS -> If job added successful
 *  @retval JOB_QUEUE_FULL -> If job queue is full
 */
int8_t job_queue_add_job(job_callback callback, uint8_t *p_data, bool ready_to_process);

/**
 * @brief This API is used to change status of the job.
 *
 *  @param[in] callback : Function provided by the user.
 *
 *  @retval JOB_QUEUE_SUCCESS -> If job callback available in the job queue
 *  @retval JOB_QUEUE_UNKNOWN_JOB_CALLBACK -> If job callback not available in the job queue
 */
int8_t job_queue_ready_to_run_job(job_callback callback);

/**
 * @brief This API is used to execute the jobs in job queue.
 *
 *  @retval JOB_QUEUE_SUCCESS -> If jobs executed successfully
 *  @retval JOB_QUEUE_EMPTY -> If jobs not available in the job queue
 */
int8_t job_queue_execute_jobs(void);

/**
 * @brief This API is used to deinit the job_queue.
 *
 */
void job_queue_deinit(void);

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
}
#endif

#endif /* JOB_QUEUE_H */