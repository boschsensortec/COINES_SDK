/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 * BSD-3-Clause
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
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
 * @file        utils.c
 *
 * @brief       Module to carry the utility functions
 *
 * @addtogroup  UTILS
 *
 * @{
 */

/******************************************************************************/
/* system header includes 													  */
/******************************************************************************/
#include <stdint.h>
#include <string.h>

/******************************************************************************/
/* own header files 														  */
/******************************************************************************/
#include "nrf.h"
#include "nrf_wdt.h"
#include "utils.h"

/******************************************************************************/
/* local macro definitions 													  */
/******************************************************************************/

/**
 * @brief RAM location to keep the magic data for BTL to perform application 
 *        switch 
 */
#define  BOOTLOADER_MAGIC_LOCATION          UINT32_C(0x2003FFF4)

/**
 * @brief Location in which application start address saved before switching 
 */
#define  BOOTLOADER_APP_START_ADDR          (BOOTLOADER_MAGIC_LOCATION + 4)

/**
 * @brief Magic data to be used for directing the bootloader to perform 
 *        application switch
 */
#define  APP_SWITCH_DIRECTIVE               "COIN"

/**
 * @brief Magic data to be used for directing the bootloader to perform 
 *        application switch
 */
#define  APP_SWITCH_DIRECTIVE_BYTE_LENGTH   UINT8_C(4)


/******************************************************************************/
/* constant definitions 												      */
/******************************************************************************/

/******************************************************************************/
/* global variables 														  */
/******************************************************************************/

/******************************************************************************/
/* static variables 														  */
/******************************************************************************/

/******************************************************************************/
/* static function declaration 												  */
/******************************************************************************/

/******************************************************************************/
/* functions 																  */
/******************************************************************************/

void utils_restart_device_to(utils_device_modes_t mode)
{     
    memcpy(((int8_t *)BOOTLOADER_MAGIC_LOCATION), APP_SWITCH_DIRECTIVE, APP_SWITCH_DIRECTIVE_BYTE_LENGTH);
    (*(uint32_t *)(BOOTLOADER_APP_START_ADDR)) = (uint32_t)mode;

    NVIC_SystemReset();
}

void utils_clear_application_switch_directive(void)
{
    (*(uint8_t *)BOOTLOADER_MAGIC_LOCATION) = UINT8_C(0);
    (*(uint32_t *)(BOOTLOADER_APP_START_ADDR)) = UINT32_C(0);
}

void utils_safe_wdt_feed(void)
{
    if (nrf_wdt_started() && nrf_wdt_request_status(NRF_WDT_RR0))
    {
        nrf_wdt_reload_request_set(NRF_WDT_RR0);
    }
}

/** @}*/
