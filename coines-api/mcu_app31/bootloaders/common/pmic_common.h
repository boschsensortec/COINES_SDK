/**
 * Copyright (C) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    pmic_common.h
 * @date    July 12, 2023
 * @brief   PMIC driver for BLD and MTP
 */

#ifndef PMIC_COMMON_H_
#define PMIC_COMMON_H_

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "nrfx_timer.h"
#include "nrf_timer.h"

/* I2C common */
#include "i2c_common.h"

/* PMIC driver */
#include "bq25120.h"

/*USB interrupt pin*/
#define VIN_DEC                 NRF_GPIO_PIN_MAP(1, 13)
/*PMIC interrupt pin*/
#define P_INT	                NRF_GPIO_PIN_MAP(1, 15)

void pmic_control_config(void);

#endif /* PMIC_COMMON_H_ */

/** @}*/