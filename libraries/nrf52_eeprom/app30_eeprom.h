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
 * @file    app30_eeprom.h
 * @date    Sep 20, 2018
 * @brief This file contains support for 1-wire EEPROM
 *
 */

#ifndef APP30_EEPROM_H_
#define APP30_EEPROM_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* functions declarations*/
/**********************************************************************************/

/*!
 *
 * @brief       : API initialize APP3.0 EEPROM
 */
void app30_eeprom_init(void);

/*!
 *
 * @brief       : API to read the internal ROM ID, and checks whether the read id is correct or not
 *
 * @param[in]   : uint8_t *buffer
 *
 * @return      : true  : read the correct rom id
 *                false : unable to read the rom id or slave is not responding
 */
bool app30_eeprom_romid(uint8_t *buffer);

/*!
 *
 * @brief       : API to read APP3.0 EEPROM
 *
 * @param[in]   : address,pointer to the buffer,length
 *
 * @return      : true/false
 */
bool app30_eeprom_read(uint16_t address, uint8_t *buffer, uint8_t length);

/*!
 *
 * @brief       : API to write APP3.0 EEPROM
 *
 * @param[in]   : address,pointer to the buffer,length
 *
 * @return      : true/false
 */
bool app30_eeprom_write(uint8_t address, uint8_t *buffer, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* APP30_EEPROM_H_ */
