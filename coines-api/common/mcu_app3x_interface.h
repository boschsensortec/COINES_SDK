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
 * @file    mcu_app3x_interface.h
 * @date    Nov 23, 2021
 * @brief   This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef MCU_APP3X_INTERFACE_H_
#define MCU_APP3X_INTERFACE_H_

#include <stdint.h>
#include <stdio.h>

#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! Interface instances*/
#define SPIM_TWIM_INSTANCE_0 0
#define SPIM_TWIM_INSTANCE_1 1
#define SPIM_TWIM_INSTANCE_2 2
#define SPIM_TWIM_INSTANCE_3 3

/*! I2C timeout in milliseconds */
#define I2C_TIMEOUT_MS               (1000)
#if defined(MCU_APP30) || defined(MCU_APP31)
/*! I2C pins for primary (Sensor) interface */
#define TWIM0_INSTANCE               0
#define I2C0_SEN_SDA_PIN             NRF_GPIO_PIN_MAP(0, 6)
#define I2C0_SEN_SCL_PIN             NRF_GPIO_PIN_MAP(0, 16)

/*! I2C pins for temperature sensor (On-board) interface */
#define TWIM1_INSTANCE               1
#define I2C1_INTERNAL_TEMP_SDA_PIN   NRF_GPIO_PIN_MAP(0, 29)
#define I2C1_INTERNAL_TEMP_SCL_PIN   NRF_GPIO_PIN_MAP(0, 26)
#elif defined(MCU_HEAR3X)
/*! I2C pins for primary (Sensor) interface */
#define TWIM0_INSTANCE               0
#define I2C0_SEN_SDA_PIN             NRF_GPIO_PIN_MAP(0, 15)
#define I2C0_SEN_SCL_PIN             NRF_GPIO_PIN_MAP(0, 24)

/*! I2C pins for temperature sensor (On-board) interface */
#define TWIM1_INSTANCE               1
#define I2C1_INTERNAL_TEMP_SDA_PIN   NRF_GPIO_PIN_MAP(0, 13)
#define I2C1_INTERNAL_TEMP_SCL_PIN   NRF_GPIO_PIN_MAP(0, 14)
#endif

/*! I2C pins for external temperature sensor (Adapter board) interface */
#define I2C1_EXTERNAL_TEMP_SDA_PIN   NRF_GPIO_PIN_MAP(0, 13)    /* GPIO PIN - 1 */
#define I2C1_EXTERNAL_TEMP_SCL_PIN   NRF_GPIO_PIN_MAP(0, 14)    /* GPIO PIN - 0 */

/*! Macro definitions for enable/disable */
#define COINES_DISABLE               UINT8_C(0x00)
#define COINES_ENABLE                UINT8_C(0x01)

/*! Macro definitions for I2C transaction status */
#define COINES_I2C_TX_NONE           UINT8_C(0)
#define COINES_I2C_TX_SUCCESS        UINT8_C(1)
#define COINES_I2C_TX_FAILED         UINT8_C(2)

/*! SPI pins for primary (Sensor) interface */
#if defined(MCU_APP30)
#define SPIM0_INSTANCE               3
#elif defined(MCU_APP31)/*MCU_APP31*/
#define SPIM0_INSTANCE               0
#endif
#if defined(MCU_APP30) || defined(MCU_APP31)
#define SPI0_SEN_MOSI_PIN            NRF_GPIO_PIN_MAP(0, 6) /* GPIO PIN - 43 */
#define SPI0_SEN_MISO_PIN            NRF_GPIO_PIN_MAP(0, 15) /* GPIO PIN - 3 */
#define SPI0_SEN_SCK_PIN             NRF_GPIO_PIN_MAP(0, 16) /* GPIO PIN - 4 */
#define SPI0_SEN_CS_PIN              NRF_GPIO_PIN_MAP(0, 24) /* GPIO PIN - 5 */

#elif defined(MCU_HEAR3X)
#define SPIM0_INSTANCE               3

#define SPI0_SEN_MOSI_PIN            NRF_GPIO_PIN_MAP(0, 15) /* GPIO PIN - 43 */
#define SPI0_SEN_MISO_PIN            NRF_GPIO_PIN_MAP(0, 16) /* GPIO PIN - 3 */
#define SPI0_SEN_SCK_PIN             NRF_GPIO_PIN_MAP(0, 24) /* GPIO PIN - 4 */
#define SPI0_SEN_CS_PIN              NRF_GPIO_PIN_MAP(0, 25) /* GPIO PIN - 5 */
#endif

/*! SPI pins for secondary (OIS) interface */
#define SPIM1_INSTANCE               3
#define SPI1_OIS_MOSI_PIN            NRF_GPIO_PIN_MAP(1, 10) /* GPIO PIN - 38 MOSI */
#define SPI1_OIS_MISO_PIN            NRF_GPIO_PIN_MAP(1, 11) /* GPIO PIN - 39 MIS0 */
#define SPI1_OIS_SCK_PIN             NRF_GPIO_PIN_MAP(1, 2) /* GPIO PIN - 36 SCLK */
#define SPI1_OIS_CS_PIN              NRF_GPIO_PIN_MAP(1, 3) /* GPIO PIN - 37 CS */

/*! SPI pins for external flash chip */
#define SPIM3_INSTANCE               2
#define SPI3_QSPI_MOSI_PIN           NRF_GPIO_PIN_MAP(0, 20) /* GPIO PIN - 50 MOSI */
#define SPI3_QSPI_MISO_PIN           NRF_GPIO_PIN_MAP(0, 21) /* GPIO PIN - 48 MIS0 */
#define SPI3_QSPI_SCK_PIN            NRF_GPIO_PIN_MAP(0, 19) /* GPIO PIN - 52 SCLK */
#define SPI3_QSPI_CS_PIN             NRF_GPIO_PIN_MAP(0, 17) /* GPIO PIN - 51 CS */


/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/
extern uint8_t multi_io_map[];

/*!
 * @brief i2c bus pin mapping
 */
enum coines_i2c_pin_map {
    COINES_I2C_PIN_DEFAULT,         /*< Default state to ignore pin mapping */
    COINES_I2C_PIN_PRIMARY,         /*< Pin mapping for primary sensor */
    COINES_I2C_PIN_INTERNAL_TEMP,   /*< Pin mapping for internal (BLE) temperature */
    COINES_I2C_PIN_SECONDARY       /*< Pin mapping for secondary sensor(AUX) */
};

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/**@brief Function returns enable/disable status of I2C bus instance.
 *
 * @param[in] bus   I2C bus instance.
 *
 *  @return Results of API execution status.
 *  @retval true -> Success
 *  @retval false -> Fail
 */
bool coines_is_i2c_enabled(enum coines_i2c_bus bus);

/**@brief Function for configuring the I2C bus with explicit pin mapping.
 *
 * @param[in] bus       :   I2C bus instance.
 * @param[in] mode      :   I2C speed mode settings.
 * @param[in] pin_map   :   I2C pin mapping.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_config_i2c_bus_internal(enum coines_i2c_bus bus,
                                       enum coines_i2c_mode i2c_mode,
                                       enum coines_i2c_pin_map pin_map);

#endif /* MCU_APP3X_INTERFACE_H_ */
