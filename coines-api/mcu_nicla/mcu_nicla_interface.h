/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    mcu_nicla_interface.h
 * @date    May 10, 2022
 * @brief   This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef MCU_NICLA_INTERFACE_H_
#define MCU_NICLA_INTERFACE_H_

#include <stdint.h>
#include <stdio.h>

#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! I2C timeout in milliseconds */
#define I2C_TIMEOUT_MS           (1000)

/*! I2C pins for LED interface */
#define TWIM0_INSTANCE          0
#define I2C0_LED_SDA_PIN        NRF_GPIO_PIN_MAP(0, 15)
#define I2C0_LED_SCL_PIN        NRF_GPIO_PIN_MAP(0, 16)

/*! I2C pins for level shifter interface */
#define TWIM1_INSTANCE          1
#define I2C1_LVL_SHIFT_SDA_PIN      NRF_GPIO_PIN_MAP(0, 22)
#define I2C1_LVL_SHIFT_SCL_PIN      NRF_GPIO_PIN_MAP(0, 23)

/*! Macro definitions for enable/disable */
#define COINES_DISABLE                  UINT8_C(0x00)
#define COINES_ENABLE                   UINT8_C(0x01)

/*! Macro definitions for I2C transaction status */
#define COINES_I2C_TX_NONE				UINT8_C(0)
#define COINES_I2C_TX_SUCCESS			UINT8_C(1)
#define COINES_I2C_TX_FAILED			UINT8_C(2)

/*! SPI pins for BHI260 sensor/external flash interface */
#define SPIM0_INSTANCE			2
#define SPI0_SEN_MOSI_PIN		NRF_GPIO_PIN_MAP(0,4)
#define SPI0_SEN_MISO_PIN		NRF_GPIO_PIN_MAP(0,5)
#define SPI0_SEN_SCK_PIN		NRF_GPIO_PIN_MAP(0,3)
#define SPI0_SEN_CS_PIN			NRF_GPIO_PIN_MAP(0,31)
#define SPI0_EXT_FLASH_CS_PIN	NRF_GPIO_PIN_MAP(0,26)

/*! SPI pins for level shifter IC */
#define SPIM1_INSTANCE			2
#define SPI1_LVL_SHIFT_MOSI_PIN		NRF_GPIO_PIN_MAP(0,27)
#define SPI1_LVL_SHIFT_MISO_PIN		NRF_GPIO_PIN_MAP(0,28)
#define SPI1_LVL_SHIFT_SCK_PIN		NRF_GPIO_PIN_MAP(0,11)
#define SPI1_LVL_SHIFT_CS_PIN		NRF_GPIO_PIN_MAP(0,29)

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/
#if 0
extern uint8_t multi_io_map[];
#endif

/*!
 * @brief i2c bus pin mapping
 */
enum coines_i2c_pin_map
{
    COINES_I2C_PIN_DEFAULT,             /*< Default state to ignore pin mapping */
    COINES_I2C_PIN_LED,                 /*< Pin mapping for LED interface */
    COINES_I2C_PIN_LEVEL_SHIFTER        /*< Pin mapping for Level shifter interface */
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
int16_t coines_config_i2c_bus_internal(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode,
                              enum coines_i2c_pin_map pin_map);

#endif /* MCU_NICLA_INTERFACE_H_ */
