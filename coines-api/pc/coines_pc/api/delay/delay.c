/*!
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
 * @file delay.c
 *
 * @brief This module provides APIs for implementing delay mechanisms.
 *
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#ifdef PLATFORM_LINUX

/* To enable usleep in glibc */
#define _DEFAULT_SOURCE
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <sys/time.h>

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#else
#include <unistd.h>
#include <ctype.h>
#endif

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "delay.h"

/*********************************************************************/
/* header files */
/*********************************************************************/
#include "coines.h"
#include "api.h"
#include "protocol.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/

/*********************************************************************/
/* constant definitions */
/*********************************************************************/

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static variables */
/*********************************************************************/

/*********************************************************************/
/* extern variables */
/*********************************************************************/

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 *
 */
void coines_delay_msec(uint32_t delay_ms)
{
#ifdef PLATFORM_WINDOWS
    Sleep(delay_ms);
#else
    uint32_t delay_microsec = (uint32_t)(delay_ms * 1000);
    usleep(delay_microsec);
#endif
}

/*!
 *  @brief This API is used for introducing a delay in microseconds
 *
 */
void coines_delay_usec(uint32_t delay_us)
{
#ifdef PLATFORM_WINDOWS
    Sleep(delay_us < 1000 ? 1 : (delay_us / 1000));
#else
    usleep(delay_us);
#endif
}

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 *
 * @return Time in milliseconds
 */
uint32_t coines_get_millis(void)
{
    struct timeval current_time;
    uint32_t millisecond = 0;

    if (gettimeofday(&current_time, NULL) == 0)
    {
        millisecond = (uint32_t)((current_time.tv_sec * 1000) + (current_time.tv_usec / 1000));
    }

    return millisecond;
}

/*!
 * @brief This API returns the number of microseconds passed since the program started
 *
 */
uint64_t coines_get_micro_sec(void)
{
    struct timeval current_time;
    uint64_t microsecond = 0;

    if (gettimeofday(&current_time, NULL) == 0)
    {
        /*lint -e732 */
        microsecond = current_time.tv_usec;
    }

    return microsecond;
}
