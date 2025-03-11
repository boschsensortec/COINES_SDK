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
 *
 *
 * @file    bridge.h
 * @brief   This file contains protocol layer function prototypes, variable declarations and Macro definitions
 */

#ifndef BRIDGE_H_
#define BRIDGE_H_

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/** @brief Number of length packets */
#define BRIDGE_PROTO_LENGTH_BYTES            2

/** @brief Bridge write command */
#define BRIDGE_WRITE_CMD                    165

/** @brief Size of the bridge packet */
#define BRIDGE_SERIAL_PACKET_SIZE           256

/** @brief Size of the bridge BLE packet */
#define BRIDGE_BLE_PACKET_SIZE              230

/** @brief Header for the bridge command */
#define BRIDGE_CMD_HEADER                    UINT8_C(0xA5)

/** @brief Header for the bridge response OK */
#define BRIDGE_RESP_OK_HEADER                UINT8_C(0x5A)

/** @brief Header for the bridge response NOK */
#define BRIDGE_RESP_NOK_HEADER               UINT8_C(0x55)

/** @brief Position of the bridge protocol header */
#define BRIDGE_PROTO_HEADER_POS              (0)

/** @brief Position of the bridge protocol length */
#define BRIDGE_PROTO_LENGTH_POS              (1)

/** @brief Position of the bridge protocol command */
#define BRIDGE_PROTO_CMD_POS                 (3)

/** @brief Position of the bridge protocol payload */
#define BRIDGE_PROTO_PAYLOAD_POS             (4)

/** @brief Position of the bridge protocol register start address */
#define BRIDGE_PROTO_REG_START_ADDR_POS      (13)

/** @brief Position of the bridge protocol register data bytes length */
#define BRIDGE_PROTO_REG_DATA_BYTES_LEN_POS  (23)

/** @brief Overhead size of the bridge protocol header */
#define BRIDGE_PROTO_HEADER_OVERHEAD          BRIDGE_PROTO_PAYLOAD_POS

/** @brief Streaming response command ID - 0x1B */
#define BRIDGE_PROTO_STREAM_RESP_CMD_ID       27 

/** @brief Streaming stop command response array */
#define BRIDGE_STREAM_STOP_RESP_ARR           {0x5A, 0x5, 0x0, 0x1A, 0x00}

/**********************************************************************************/
/* function prototype declarations*/
/**********************************************************************************/

#endif /* BRIDGE_H_ */
