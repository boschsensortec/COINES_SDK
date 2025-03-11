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
 * @file    i2c_common.c
 * @date    October 07, 2022
 * @brief   I2C communication common APIs for led and pmic
 *
 */

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/
#include "i2c_common.h"


/*! Macros to define communication interfaces */

#if (!(defined(MCU_NICLA) || defined(MCU_APP31) || defined(MCU_HEAR3X)))

/*! Comm interface - I2C */
#define NICLA_I2C_INTF              UINT8_C(1)
#define NICLA_I2C_TIMEOUT_MS            (1000)

/*! Nicla I2C pins for BUS 0*/
#define TWIM1_INSTANCE           1
#define NICLA_I2C_SDA_PIN        NRF_GPIO_PIN_MAP(0, 29) //Need to be updated since this is used by NICLA and AB3.1 bootloader and mtp, not using the same pins!
#define NICLA_I2C_SCL_PIN        NRF_GPIO_PIN_MAP(0, 26) //Need to be updated since this is used by NICLA and AB3.1 bootloader and mtp, not using the same pins!

/*! Macro definitions for I2C transaction status */
#define NICLA_I2C_TXRX_NONE       UINT8_C(0)
#define NICLA_I2C_TXRX_SUCCESS    UINT8_C(1)
#define NICLA_I2C_TXRX_FAILED     UINT8_C(2)

/* Indicates if operation on I2C has completed. */
static volatile uint8_t nicla_i2c_txrx_status = NICLA_I2C_TXRX_FAILED;

/* Nicla I2C instance */
static const nrfx_twim_t nicla_i2c_instance = NRFX_TWIM_INSTANCE(TWIM1_INSTANCE);

/* I2C event handler callback prototype */
static void nicla_i2c0_event_handler(nrfx_twim_evt_t const *p_event, void *p_context);

/*!
 * @brief   This function manages the I2C0 event call back
 */
static void nicla_i2c0_event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    switch (p_event->type)
    {
    case NRFX_TWIM_EVT_DONE:
        nicla_i2c_txrx_status = NICLA_I2C_TXRX_SUCCESS;
        break;
    default:
        nicla_i2c_txrx_status = NICLA_I2C_TXRX_FAILED;
        break;
    }
}

#endif

#if defined(MCU_APP31) || defined(MCU_HEAR3X)
/* I2C communication device bus */
enum coines_i2c_bus i2c_dev_bus = COINES_I2C_BUS_1;
#elif defined(MCU_NICLA)
/* I2C communication device bus */
enum coines_i2c_bus i2c_dev_bus = COINES_I2C_BUS_0;
#endif


/*!
 * @brief  This function initialize common i2c bus
 */
int8_t common_i2c_init(void)
{
#if (!(defined(MCU_NICLA) || defined(MCU_APP31) || defined(MCU_HEAR3X)))
    int8_t rslt = NRFX_ERROR_NULL;
    nrfx_twim_config_t i2c_config = NRFX_TWIM_DEFAULT_CONFIG;

    i2c_config.scl = NICLA_I2C_SCL_PIN;
    i2c_config.sda = NICLA_I2C_SDA_PIN;
    i2c_config.frequency = NRF_TWIM_FREQ_400K;

    /*nrf_gpio_cfg(i2c_config.sda,
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_H0D1,
                 NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(i2c_config.scl,
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_H0D1,
                 NRF_GPIO_PIN_NOSENSE);*/

    if (nrfx_twim_init(&nicla_i2c_instance, &i2c_config, nicla_i2c0_event_handler,
                       NULL) == NRFX_SUCCESS)
    {

        nrfx_twim_enable(&nicla_i2c_instance);

        nrf_twim_frequency_set(nicla_i2c_instance.p_twim, i2c_config.frequency);

        rslt = NRFX_SUCCESS;
    }
    return rslt;
#else
    return coines_config_i2c_bus(i2c_dev_bus, COINES_I2C_FAST_MODE);
#endif
}

/*!
 * @brief       This function write data via I2C
 *
 * @param[in]   reg_addr : register address
 * @param[in]   reg_data : transmission buffer
 * @param[in]   length    : transmission buffer size
 * @param[in]   i2c_dev_address : device i2c address
 *
 */
int8_t common_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, uint8_t i2c_dev_address)
{
#if (!(defined(MCU_NICLA) || defined(MCU_APP31) || defined(MCU_HEAR3X)))
    nrfx_err_t error_val = NRFX_SUCCESS;

    uint8_t buffer[length + 1];

    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, length);

    nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(i2c_dev_address, buffer, (length + 1));

    nicla_i2c_txrx_status = NICLA_I2C_TXRX_NONE;
    error_val = nrfx_twim_xfer(&nicla_i2c_instance, &write_desc, NRFX_TWIM_FLAG_TX_POSTINC);

    /* Timeout the I2C operation after 1000 ms */
    volatile uint32_t timeout_ms = 100;
    while ((timeout_ms < NICLA_I2C_TIMEOUT_MS) && (nicla_i2c_txrx_status == NICLA_I2C_TXRX_NONE))
    {
        nrf_delay_ms(1);
        timeout_ms += 1;
    }

    /* If I2C transfer has timed out, recover the I2C bus */
    if ((nicla_i2c_txrx_status != NICLA_I2C_TXRX_SUCCESS) || (error_val != NRFX_SUCCESS))
    {
        error_val = NRFX_ERROR_TIMEOUT;
    }
    return error_val;
#else
    return coines_write_i2c(i2c_dev_bus, i2c_dev_address, reg_addr, (uint8_t *)reg_data,length);
#endif
}

/*!
 * @brief       This function read data via I2C
 *
 * @param[in]   reg_addr : register address
 * @param[in]   reg_data : receiver buffer
 * @param[in]   length    : receiver buffer size
 * @param[in]   i2c_dev_address : device i2c address
 *
 */
int8_t common_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, uint8_t i2c_dev_address)
{
#if (!(defined(MCU_NICLA) || defined(MCU_APP31) || defined(MCU_HEAR3X)))
    nrfx_err_t error_val = NRFX_SUCCESS;

    nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_TXRX(i2c_dev_address, &reg_addr, 1, reg_data, length);

    nicla_i2c_txrx_status = NICLA_I2C_TXRX_NONE;
    error_val = nrfx_twim_xfer(&nicla_i2c_instance, &read_desc,
                               NRFX_TWIM_FLAG_RX_POSTINC | NRFX_TWIM_FLAG_REPEATED_XFER);

    /* Timeout the I2C operation after 1000 ms */
    volatile uint32_t timeout_ms = 100;
    while ((timeout_ms < NICLA_I2C_TIMEOUT_MS) && (nicla_i2c_txrx_status == NICLA_I2C_TXRX_NONE))
    {
        nrf_delay_ms(1);
        timeout_ms += 1;
    }

    /* If I2C transfer has timed out, recover the I2C bus */
    if ((nicla_i2c_txrx_status != NICLA_I2C_TXRX_SUCCESS) || (error_val != NRFX_SUCCESS))
    {
        error_val = NRFX_ERROR_TIMEOUT;
    }
    return error_val;    
#else
    return coines_read_i2c(i2c_dev_bus, i2c_dev_address, reg_addr, reg_data,length);
#endif
}

/*!
 * @brief       This function perform delay in milliseconds
 *
 * @param[in]   period : delay period in milliseconds
 *
 */
void common_delay_ms(uint32_t period)
{
#if (!(defined(MCU_NICLA) || defined(MCU_APP31) || defined(MCU_HEAR3X)))
    nrf_delay_ms(period);
#else
    coines_delay_msec(period);
#endif
}

/*!
 * @brief  This function configures the pmic chip disable pin and set initial state
 */
int8_t common_pmic_cd_init(void)
{
#if (!(defined(MCU_NICLA) || defined(MCU_APP31) || defined(MCU_HEAR3X)))
    nrf_gpio_cfg_output(PMIC_CD);
    nrf_gpio_pin_write(PMIC_CD, 1);
    return NRFX_SUCCESS;
#else
    return coines_set_pin_config(PMIC_CD, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif
}

/*!
 * @brief  This function control chip disable pin
 */
int8_t common_pmic_cd_set(uint8_t pin_state)
{
    if(pin_state>1)
        pin_state = 1;

#if (!(defined(MCU_NICLA) || defined(MCU_APP31) || defined(MCU_HEAR3X)))
    int8_t rslt = NRFX_SUCCESS;
    nrf_gpio_pin_write(PMIC_CD, pin_state);
    return rslt;
#else
    return coines_set_pin_config(PMIC_CD, COINES_PIN_DIRECTION_OUT, pin_state);
#endif
}
