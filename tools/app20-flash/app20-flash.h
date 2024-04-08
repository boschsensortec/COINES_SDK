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
 * @file    app20-flash.h
 * @date    Oct 1, 2019
 * @version 1.0 
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "comm_intf.h"
#include "coines_defs.h"
#include "coines.h"

#define BOOT_MODE_SET (0xB1)
#define BOOT_MODE_GET (0xB2)

#define ERASE_CMD   (0x01)
#define FLASH_CMD   (0x02)
#define COMTEST_CMD (0x03)
#define MODE_CMD    (0x04)
#define LOCK_CMD    (0x05)
#define UNLOCK_CMD  (0x06)
#define UPDATE_SUCCESS_CMD (0x07)
#define DATA_CMD (0x08)
#define GETBOARDINFO_CMD  (0x09)

#define APP20_BOARD (3)
#define BNO_USB_STICK (4)

#define PAYLOAD_SIZE (0x32)
#define WRITE_SIZE (512)
#define NUM_OF_PACKETS (0x0B)
#define MAX_FILE_SIZE_BNO_STICK (63*1024)
#define MAX_FILE_SIZE_APP20 (960*1024)

#define PROGRESS_BAR_SIZE (25)

static void app2_check_mode();
static uint8_t app2_read_board_type();
static void app2_erase();
static void app2_flash();
static void app2_send_fw_data(uint8_t *data, uint8_t packet_no, uint8_t len, bool is_last);
static void app2_update_success();

static void file_read_by_len(FILE *file_h, uint8_t *fw_data, uint32_t len);
static int get_file_size(FILE *file_h);
static uint8_t get_checksum(uint8_t *fw_data, uint32_t len);
