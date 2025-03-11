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
 * @file   ble_windows.c
 * @brief  This module provides Windows specific APIs for BLE comm
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdbool.h>
#include <time.h>

/*********************************************************************/
/* own header files */
/**********************************************************************/
#include "ble_windows.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
#define BLE_TX_NOTIFY_TIMEOUT  5

/*********************************************************************/
/* Extern variables */
/**********************************************************************/
extern bool has_tx_notified;

/*********************************************************************/
/* Functions */
/*********************************************************************/

/*!
 * @brief Handles dll initialization
 */
bool handle_dll_init(void)
{
    return init_dll_loader();
}

/*!
 * @brief Handles dll deinitialization
 */
void handle_dll_deinit(void)
{
    deinit_dll();
}

/*!
 * @brief Timeout for TX notify to happen for a write_request
 */
void wait_for_tx_notify(void)
{
    time_t start = time(NULL);

    while (!has_tx_notified)
    {
        if (difftime(time(NULL), (long)start) >= BLE_TX_NOTIFY_TIMEOUT)
        {
            break;
        }

        Sleep(1);
    }
}
