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
 *  @file   coines_common.h
 *  @brief  This file contains common functions and declarations for LEGACY and COINES Bridge
 */

#ifndef COINES_COMMON_H
#define COINES_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* includes */
/**********************************************************************************/
#include "coines.h"

/*********************************************************************/
/* Typedef definitions */
/**********************************************************************/
typedef struct
{
    int16_t code;
    const char *message;
} error_code_mapping;

/*********************************************************************/
/* Macro definitions */
/**********************************************************************/


/**********************************************************************************/
/* Function declarations */
/**********************************************************************************/

/*!
 * @brief This API is used to COINES_SDK error codes to error strings
 *
 * @param[in]  error_code     : error_code
 *
 * @return Error string for the given error code
 */
const char *get_coines_error_str(int16_t error_code);

#ifdef __cplusplus
}
#endif

#endif
