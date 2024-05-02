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
 */

#ifndef COINES_BRIDGE_CLIENT_H
#define COINES_BRIDGE_CLIENT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef COINES_PACKET_SIZE
#define COINES_PACKET_SIZE                   2060
#endif

#ifndef COINES_BUFFER_SIZE
#define COINES_BUFFER_SIZE                   2060
#endif

/*! Maximum header length for legacy and new protocol */
#define COINES_MAX_HEADER_LEN                UINT8_C(12)

#define COINES_CMD_HEADER                    UINT8_C(0xA5)
#define COINES_RESP_OK_HEADER                UINT8_C(0x5A)
#define COINES_RESP_NOK_HEADER               UINT8_C(0x55)

#define COINES_PROTO_HEADER_POS              (0)
#define COINES_PROTO_LENGTH_POS              (1)
#define COINES_PROTO_CMD_POS                 (3)
#define COINES_PROTO_PAYLOAD_POS             (4)
#define COINES_PROTO_REG_START_ADDR_POS      (13)
#define COINES_PROTO_REG_DATA_BYTES_LEN_POS  (23)

enum coines_cmds {
    COINES_CMD_ID_ECHO,
    COINES_CMD_ID_GET_BOARD_INFO,
    COINES_CMD_ID_SET_PIN,
    COINES_CMD_ID_GET_PIN,
    COINES_CMD_ID_SET_VDD_VDDIO,
    COINES_CMD_ID_SPI_CONFIG,
    COINES_CMD_ID_SPI_DECONFIG,
    COINES_CMD_ID_SPI_WORD_CONFIG,
    COINES_CMD_ID_SPI_WRITE_REG_16,
    COINES_CMD_ID_SPI_WRITE_REG,
    COINES_CMD_ID_SPI_READ_REG_16,
    COINES_CMD_ID_SPI_READ_REG,
    COINES_CMD_ID_I2C_CONFIG,
    COINES_CMD_ID_I2C_DECONFIG,
    COINES_CMD_ID_I2C_WRITE_REG,
    COINES_CMD_ID_I2C_READ_REG,
    COINES_CMD_ID_I2C_WRITE,
    COINES_CMD_ID_I2C_READ,
    COINES_CMD_ID_GET_TEMP,
    COINES_CMD_ID_GET_BATTERY,
    COINES_CMD_ID_RESET,
    COINES_CMD_ID_SET_LED,
    COINES_CMD_ID_POLL_STREAM_COMMON,
    COINES_CMD_ID_POLL_STREAM_CONFIG,
    COINES_CMD_ID_INT_STREAM_CONFIG,
    COINES_CMD_ID_FIFO_STREAM_CONFIG,
    COINES_CMD_ID_STREAM_START_STOP,
    COINES_READ_SENSOR_DATA,
    COINES_CMD_ID_SOFT_RESET,
    COINES_CMD_ID_SHUTTLE_EEPROM_WRITE,
    COINES_CMD_ID_SHUTTLE_EEPROM_READ,
    COINES_N_CMDS
};

typedef enum sensor_interface {
    COINES_I2C,
    COINES_SPI
} sensor_interface_t;

typedef int8_t (*coines_cmd_callback)(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                                      uint16_t *resp_length);

struct coines_cbt
{
    coines_cmd_callback cmd_callback[COINES_N_CMDS];
};

int8_t coines_process_packet(uint8_t *packet,
                             uint16_t packet_length,
                             uint8_t *resp,
                             uint16_t *resp_length,
                             struct coines_cbt *cbt);

#ifdef __cplusplus
}
#endif

#endif
