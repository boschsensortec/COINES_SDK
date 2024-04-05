/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
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