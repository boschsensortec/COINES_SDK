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
 * @file    i2c_common.h
 * @date    October 07, 2022
 * @brief   I2C driver for led and pmic
 */

#ifndef COMMON_I2C_COMMON_H_
#define COMMON_I2C_COMMON_H_

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "nrf_gpio.h"
#if (!(defined(MCU_NICLA) || defined(MCU_APP31)))
#include "nrfx_twim.h"
#include "nrf_twim.h"
#include "nrf_delay.h"
#define PMIC_CD                        NRF_GPIO_PIN_MAP(0, 28)   /* GPIO_30/GPIO_31 */
#else
#include "coines.h"
#if defined(MCU_NICLA)
#define PMIC_CD                        COINES_NICLA_CD_PIN
#elif defined(MCU_APP31)
#define PMIC_CD                        COINES_APP31_CD
#endif
#endif

int8_t common_i2c_init(void);
int8_t common_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, uint8_t i2c_dev_address);
int8_t common_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, uint8_t i2c_dev_address);
void common_delay_ms(uint32_t period);
int8_t common_pmic_cd_init(void);
int8_t common_pmic_cd_set(uint8_t pin_state);

#endif /* COMMON_I2C_COMMON_H_ */

/** @}*/