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
 * @file    coines_bridge_firmware.h
 * @brief   This file contains function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef COINES_BRIDGE_FIRMWARE_H_
#define COINES_BRIDGE_FIRMWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/* COM_RW_BUFF_SIZE - Configurable in compile time through makefile, default value is 2048 */
#define READ_BUFF_SIZE                    UINT16_C(COM_RW_BUFF_SIZE + COINES_MAX_HEADER_LEN)

#define WRITE_BUFF_SIZE                   UINT16_C(COM_RW_BUFF_SIZE + COINES_MAX_HEADER_LEN + TIMESTAMP_SIZE)

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/



#ifdef __cplusplus
}
#endif

#endif /* COINES_BRIDGE_FIRMWARE_H_ */
