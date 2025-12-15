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
 * @file        utils.h
 *
 * @brief       This is the header file which carry the utility methods and
 *              definitions.
 *
 * @addtogroup 	UTILS
 *
 * @{
 */

#ifndef _UTILS_H
#define _UTILS_H

/******************************************************************************/
/* header includes                                                            */
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/* macro definitions														  */
/******************************************************************************/
/**
 * @brief Macro to represent the address at which the DFU bootloader application 
 *        resides
 */
#define DFU_BOOTLOADER_ADDR    UINT32_C(0x00000)

/**
 * @brief Macro to represent the address at which this USB MTP application 
 *        resides
 */
#define MTP_FIRMWARE_ADDR      UINT32_C(0x28000)

/******************************************************************************/
/* type definitions															  */
/******************************************************************************/

/**
 * @brief Enumeration to represent different application modes at which this
 *        device, the application board, can execute
 */
typedef enum utils_device_modes_e
{
    DFU_MODE = DFU_BOOTLOADER_ADDR,
    MTP_MODE = MTP_FIRMWARE_ADDR
} utils_device_modes_t;

/******************************************************************************/
/* (extern) variable declarations											  */
/******************************************************************************/

/******************************************************************************/
/* function prototype declarations 											  */
/******************************************************************************/

/**
 * @brief Utility method to restart the device into specific application mode 
 *        which is specified through the argument
 * 
 * @param[in] mode : the application mode to which the device need to boot after
 *                   the restart
 */
void utils_restart_device_to(utils_device_modes_t mode);

/**
 * @brief Utility method to clear any existing application switch directive set 
 *        for the bootloader
 */
void utils_clear_application_switch_directive(void);

/******************************************************************************/
/* inline function definitions												  */
/******************************************************************************/



#endif // _UTILS_H

/** @}*/
