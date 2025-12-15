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
 * @file    mcu_app3x_support.c
 * @date    May 25, 2021
 * @brief   COINES_SDK support file for mcu_app30.c
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "mcu_app3x_interface.h"
#include "coines_common.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
/* idx = 0 ==> Primary sensor I2C/SPI interface; User configurable */

/* idx = 1 ==> Ble temperature I2C interface; Enabled internally but
 *              can be reconfigured to other sensors if requested
 *              e.g. adapter board, since the on-board (BLE)
 *              temperature sensor has low priority
 *              For APP3.1 and hearable board, PMIC is connected */

/* idx = 2 ==> External flash SPI interface; User configurable */
/* idx = 3 ==> Secondary sensor SPI interface; User configurable */

/* 

### Instance & Bus Allocation for APP3.1  

| Flash Chip (Instance)| PMIC (Instance)    | Prim Sensor (Instance) | Sec Sensor (Instance)  |
|----------------------|--------------------|------------------------|------------------------|
| SPI_BUS_INT (SPIM2)  | I2C_BUS_INT (TWI1) | SPI_BUS_0 (SPIM3)      |                        |
| SPI_BUS_INT (SPIM2)  | I2C_BUS_INT (TWI1) |                        | SPI_BUS_1 (SPIM3)      |
| SPI_BUS_INT (SPIM2)  | I2C_BUS_INT (TWI1) | I2C_BUS_0 (TWI0)       |                        |
| SPI_BUS_INT (SPIM2)  | I2C_BUS_INT (TWI1) | SPI_BUS_0 (SPIM3)      | I2C_BUS_1 (TWI0)       |
| SPI_BUS_INT (SPIM2)  | I2C_BUS_INT (TWI1) | I2C_BUS_0 (TWI0)       | SPI_BUS_1 (SPIM3)      |


*/
uint8_t coines_spi_twi_instances[4] = { 0, 0, 0, 0 };

/**********************************************************************************/
/* static function declartions */
/**********************************************************************************/
static void coines_i2c0_event_handler(nrfx_twim_evt_t const *p_event, void *p_context);
static void coines_i2c1_event_handler(nrfx_twim_evt_t const *p_event, void *p_context);
static void coines_i2c_int_event_handler(nrfx_twim_evt_t const *p_event, void *p_context);
static int16_t coines_i2c_bus_recover(enum coines_i2c_bus bus);
static void coines_get_i2c_pin_map(enum coines_i2c_bus bus, enum coines_i2c_pin_map pin_map);
int16_t coines_set_spi_instance(enum coines_spi_bus bus, uint8_t enable);
static int16_t coines_set_i2c_instance(enum coines_i2c_bus bus, uint8_t enable);
bool coines_is_spi_enabled(enum coines_spi_bus bus);
uint8_t prepare_reg_addr_buffer(uint8_t *reg_addr_buffer, uint16_t reg_addr, uint8_t transfer_bits);

/**********************************************************************************/
/* static variables */
/**********************************************************************************/

static const nrfx_twim_config_t i2c_primary_pin_config = {
    .scl = I2C0_SEN_SCL_PIN, .sda = I2C0_SEN_SDA_PIN, .frequency = NRF_TWIM_FREQ_400K,
    .interrupt_priority = (uint8_t)APP_IRQ_PRIORITY_HIGH, .hold_bus_uninit = false
};
static const nrfx_twim_config_t i2c_secondary_pin_config = {
    .scl = I2C1_SEN_SCL_PIN, .sda = I2C1_SEN_SDA_PIN, .frequency = NRF_TWIM_FREQ_400K,
    .interrupt_priority = (uint8_t)APP_IRQ_PRIORITY_HIGH, .hold_bus_uninit = false
};
static const nrfx_twim_config_t i2c_internal_pin_config = {
    .scl = I2C_INTERNAL_SCL_PIN, .sda = I2C_INTERNAL_SDA_PIN, .frequency = NRF_TWIM_FREQ_400K,
    .interrupt_priority = (uint8_t)APP_IRQ_PRIORITY_HIGH, .hold_bus_uninit = false
};

coines_i2c_intf_t coines_i2c_intf[COINES_I2C_BUS_MAX] = {
    /* COINES_I2C_BUS_0 - TWI0 PRIMARY PINS */
    { .instance = TWIM0_INSTANCE, .peripheral_instance = NRFX_TWIM0_INSTANCE, .enabled = false,
      .txrx_status = COINES_I2C_TX_FAILED, .config = i2c_primary_pin_config,
      .event_handler = coines_i2c0_event_handler },

    /* COINES_I2C_BUS_1 - TWI0 SECONDARY PINS */
    { .instance = TWIM0_INSTANCE, .peripheral_instance = NRFX_TWIM0_INSTANCE, .enabled = false,
      .txrx_status = COINES_I2C_TX_FAILED, .config = i2c_secondary_pin_config,
      .event_handler = coines_i2c1_event_handler },

    /* COINES_I2C_BUS_INT - TWI1 INTERNAL PINS */
    { .instance = TWIM1_INSTANCE, .peripheral_instance = NRFX_TWIM1_INSTANCE, .enabled = false,
      .txrx_status = COINES_I2C_TX_FAILED, .config = i2c_internal_pin_config,
      .event_handler = coines_i2c_int_event_handler }
};

static const nrfx_spim_config_t spi_primary_pin_config = {
    .sck_pin = SPI0_SEN_SCK_PIN, .mosi_pin = SPI0_SEN_MOSI_PIN, .miso_pin = SPI0_SEN_MISO_PIN,
    .ss_pin = NRFX_SPIM_PIN_NOT_USED, .irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY, .orc = 0xFF,
    .frequency = NRF_SPIM_FREQ_4M, .mode = NRF_SPIM_MODE_0, .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
    NRFX_SPIM_DEFAULT_EXTENDED_CONFIG
};

static const nrfx_spim_config_t spi_secondary_pin_config = {
    .sck_pin = SPI1_SEN_SCK_PIN, .mosi_pin = SPI1_SEN_MOSI_PIN, .miso_pin = SPI1_SEN_MISO_PIN,
    .ss_pin = NRFX_SPIM_PIN_NOT_USED, .irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY, .orc = 0xFF,
    .frequency = NRF_SPIM_FREQ_4M, .mode = NRF_SPIM_MODE_0, .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
    NRFX_SPIM_DEFAULT_EXTENDED_CONFIG
};

static const nrfx_spim_config_t spi_internal_pin_config = {
    .sck_pin = SPI_FLASH_SCK_PIN, .mosi_pin = SPI_FLASH_MOSI_PIN, .miso_pin = SPI_FLASH_MISO_PIN,
    .ss_pin = NRFX_SPIM_PIN_NOT_USED, .irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY, .orc = 0xFF,
    .frequency = NRF_SPIM_FREQ_4M, .mode = NRF_SPIM_MODE_0, .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
    NRFX_SPIM_DEFAULT_EXTENDED_CONFIG
};

coines_spi_intf_t coines_spi_intf[COINES_SPI_BUS_MAX] = {
    /* COINES_SPI_BUS_0 - SPIM3 PRIMARY PINS */
    {
        .instance = SPIM3_INSTANCE, .peripheral_instance = NRFX_SPIM3_INSTANCE,
        .enabled = false,
        .config = spi_primary_pin_config,
        .txrx_desc = { .p_tx_buffer = NULL, .tx_length = 0, .p_rx_buffer = NULL, .rx_length = 0 }
    },

    /* COINES_SPI_BUS_1 - SPIM3 SECONDARY PINS */
    { .instance = SPIM3_INSTANCE, .peripheral_instance = NRFX_SPIM3_INSTANCE, .enabled = false,
      .config = spi_secondary_pin_config,
      .txrx_desc = { .p_tx_buffer = NULL, .tx_length = 0, .p_rx_buffer = NULL, .rx_length = 0 } },

    /* COINES_SPI_BUS_INT - SPIM2 INTERNAL PINS */
    { .instance = SPIM2_INSTANCE, .peripheral_instance = NRFX_SPIM2_INSTANCE, .enabled = false,
      .config = spi_internal_pin_config,
      .txrx_desc = { .p_tx_buffer = NULL, .tx_length = 0, .p_rx_buffer = NULL, .rx_length = 0 } }
};
/**********************************************************************************/
/* Prototypes */
/**********************************************************************************/

/** @brief Callback function invoked when an I2C bus transaction is completed. */
__attribute__((weak)) void coines_i2c_bus_transaction_complete_callback(enum coines_i2c_bus bus, bool status);
/**********************************************************************************/
/* Functions */
/**********************************************************************************/
/*!
 * @brief   This function manages the I2C0 event call back
 */
static void coines_i2c0_event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    coines_i2c_bus_transaction_complete_callback(COINES_I2C_BUS_0 , (p_event->type == NRFX_TWIM_EVT_DONE) ? true : false );
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            coines_i2c_intf[COINES_I2C_BUS_0].txrx_status = COINES_I2C_TX_SUCCESS;
            break;
        default:
            coines_i2c_intf[COINES_I2C_BUS_0].txrx_status = COINES_I2C_TX_FAILED;
            break;
    }
}

/*!
 * @brief   This function manages the I2C1 event call back
 */
static void coines_i2c1_event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    coines_i2c_bus_transaction_complete_callback(COINES_I2C_BUS_1 , (p_event->type == NRFX_TWIM_EVT_DONE) ? true : false );
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            coines_i2c_intf[COINES_I2C_BUS_1].txrx_status = COINES_I2C_TX_SUCCESS;
            break;
        default:
            coines_i2c_intf[COINES_I2C_BUS_1].txrx_status = COINES_I2C_TX_FAILED;
            break;
    }
}

/*!
 * @brief   This function manages the I2C_INT event call back
 */
static void coines_i2c_int_event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    coines_i2c_bus_transaction_complete_callback(COINES_I2C_BUS_INT , (p_event->type == NRFX_TWIM_EVT_DONE) ? true : false );
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            coines_i2c_intf[COINES_I2C_BUS_INT].txrx_status = COINES_I2C_TX_SUCCESS;
            break;
        default:
            coines_i2c_intf[COINES_I2C_BUS_INT].txrx_status = COINES_I2C_TX_FAILED;
            break;
    }
}

/*!
 * @brief   This function returns the I2C bus enabled status
 */
bool coines_is_i2c_enabled(enum coines_i2c_bus bus)
{
    return coines_i2c_intf[bus].enabled ? true : false;
}

/*!
 * @brief : API to recover I2C Bus
 */
static int16_t coines_i2c_bus_recover(enum coines_i2c_bus bus)
{
    nrfx_err_t error = NRFX_SUCCESS;
    int16_t result = COINES_SUCCESS;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            /* nRF5 SDK v16+ has nrfx_twim_bus_recover() */
            nrfx_twim_uninit(&coines_i2c_intf[bus].peripheral_instance);
            coines_i2c_intf[bus].enabled = false; /* Set I2C bus status to disabled */

            /* TODO: Re-initialize originally set I2C speed ! */
            error = nrfx_twim_init(&coines_i2c_intf[bus].peripheral_instance,
                                   &coines_i2c_intf[bus].config,
                                   coines_i2c_intf[bus].event_handler,
                                   NULL);
            nrfx_twim_enable(&coines_i2c_intf[bus].peripheral_instance);
            if (NRFX_SUCCESS != error)
            {
                printf("I2C recovery failed\n\r");
                result = COINES_E_FAILURE;
            }
            else
            {
                coines_i2c_intf[bus].enabled = true; /* Enabled I2C bus after recovery */
                result = COINES_SUCCESS;
            }
        }
        else
        {
            result = COINES_E_I2C_BUS_NOT_ENABLED;
        }
    }
    else
    {
        result = COINES_E_I2C_INVALID_BUS_INTF;
    }

    return result;
}

/*!
 * @brief   This function returns the SPI bus enabled status
 */
bool coines_is_spi_enabled(enum coines_spi_bus bus)
{
    return coines_spi_intf[bus].enabled ? true : false;
}

/*!
 * @brief   This function returns the I2C instance status
 */
static int16_t coines_get_i2c_instance(enum coines_i2c_bus bus)
{
    int16_t return_val = COINES_SUCCESS;
    uint8_t instance_idx = coines_i2c_intf[bus].instance;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {   
        return_val = coines_spi_twi_instances[instance_idx];
    }
    else
    {
        return_val = COINES_E_I2C_INVALID_BUS_INTF;
    }

    return return_val;
}

/*!
 * @brief   This function sets the I2C instance status
 */
static int16_t coines_set_i2c_instance(enum coines_i2c_bus bus, uint8_t enable)
{
    int16_t return_val = COINES_SUCCESS;
    uint8_t instance_idx = coines_i2c_intf[bus].instance;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {   
        if (COINES_ENABLE == enable)
        {
            if (0 != coines_spi_twi_instances[instance_idx])
            {
                return COINES_E_I2C_CONFIG_EXIST;
            }

            coines_spi_twi_instances[instance_idx] = COINES_ENABLE;
        }
        else if (COINES_DISABLE == enable)
        {
            coines_spi_twi_instances[instance_idx] = COINES_DISABLE;
        }
        else
        {
            return_val = COINES_E_NOT_SUPPORTED;
        }
    }
    else
    {
        return_val = COINES_E_I2C_INVALID_BUS_INTF;
    }

    return return_val;
}

/*!
 * @brief   This function returns the SPI instance status
 */
int16_t coines_get_spi_instance(enum coines_spi_bus bus)
{
    int16_t return_val = COINES_SUCCESS;
    uint8_t instance_idx = coines_spi_intf[bus].instance;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {

        return_val = coines_spi_twi_instances[instance_idx];
    }
    else
    {
        return_val = COINES_E_SPI_INVALID_BUS_INTF;
    }

    return return_val;
}

/*!
 * @brief   This function sets the SPI instance status
 */
int16_t coines_set_spi_instance(enum coines_spi_bus bus, uint8_t enable)
{
    int16_t return_val = COINES_SUCCESS;
    uint8_t instance_idx = coines_spi_intf[bus].instance;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (COINES_ENABLE == enable)
        {
            if (0 != coines_spi_twi_instances[instance_idx])
            {
                return COINES_E_SPI_CONFIG_EXIST;
            }

            coines_spi_twi_instances[instance_idx] = COINES_ENABLE;
        }
        else if (COINES_DISABLE == enable)
        {
            coines_spi_twi_instances[instance_idx] = COINES_DISABLE;
        }
        else
        {
            return_val = COINES_E_NOT_SUPPORTED;
        }
    }
    else
    {
        return_val = COINES_E_SPI_INVALID_BUS_INTF;
    }

    return return_val;
}

/**
 * @brief Prepares the register address in the transmit buffer based on the transfer bit size.
 */
uint8_t prepare_reg_addr_buffer(uint8_t *reg_addr_buffer, uint16_t reg_addr, uint8_t transfer_bits) 
{
    uint8_t reg_addr_buffer_len = 0;

    if (transfer_bits == COINES_SPI_TRANSFER_8BIT || transfer_bits == COINES_I2C_TRANSFER_8BIT) 
    {
        reg_addr_buffer[0] = (uint8_t)reg_addr;  
        reg_addr_buffer_len = 1;
    } 
    else if (transfer_bits == COINES_SPI_TRANSFER_16BIT || transfer_bits == COINES_I2C_TRANSFER_16BIT) 
    {
        reg_addr_buffer[0] = (uint8_t)(reg_addr >> 8);  // MSB
        reg_addr_buffer[1] = (uint8_t)(reg_addr & 0xFF); // LSB
        reg_addr_buffer_len = 2;
    }
    return reg_addr_buffer_len;
}

/*!
 *  @brief This API is used to configure the SPI bus
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
{
    int16_t retval = COINES_SUCCESS;

    if (COINES_SUCCESS != coines_get_spi_instance(bus))
    {   
        return COINES_E_SPI_CONFIG_FAILED;
    }


    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (!coines_is_spi_enabled(bus)) /* check whether SPI bus is already enabled */
        {
            coines_spi_intf[bus].config.mode = (nrf_spim_mode_t)spi_mode;

#define COINES_NRF_SPEED_MAP(coines_spi, nrf_spi)  \
        case  COINES_SPI_SPEED_##coines_spi:         \
            coines_spi_intf[bus].config.frequency = NRF_SPIM_FREQ_##nrf_spi; \
            break \
            
#define COINES_NRF_PLATFORM_SPEED_MAP(coines_spi, nrf_spi)  \
        case  COINES_SPI_PLATFORM_SPEED_##coines_spi:         \
            coines_spi_intf[bus].config.frequency = NRF_SPIM_FREQ_##nrf_spi; \
            break \

            switch (spi_speed)
            {
            COINES_NRF_SPEED_MAP(250_KHZ, 250K);
            COINES_NRF_SPEED_MAP(300_KHZ, 250K);

            COINES_NRF_SPEED_MAP(400_KHZ, 500K);
            COINES_NRF_SPEED_MAP(500_KHZ, 500K);
            COINES_NRF_SPEED_MAP(600_KHZ, 500K);

            COINES_NRF_SPEED_MAP(750_KHZ, 1M);
            COINES_NRF_SPEED_MAP(1_MHZ, 1M);
            COINES_NRF_SPEED_MAP(1_2_MHZ, 1M);
            COINES_NRF_SPEED_MAP(1_25_MHZ, 1M);

            COINES_NRF_SPEED_MAP(1_5_MHZ, 2M);
            COINES_NRF_SPEED_MAP(2_MHZ, 2M);
            COINES_NRF_SPEED_MAP(2_5_MHZ, 2M);

            COINES_NRF_SPEED_MAP(3_MHZ, 4M);
            COINES_NRF_SPEED_MAP(3_75_MHZ, 4M);
            COINES_NRF_SPEED_MAP(5_MHZ, 4M);
            COINES_NRF_SPEED_MAP(6_MHZ, 4M);
            COINES_NRF_SPEED_MAP(7_5_MHZ, 4M);

            COINES_NRF_SPEED_MAP(8_MHZ, 8M);
            COINES_NRF_SPEED_MAP(10_MHZ, 8M);

            COINES_NRF_PLATFORM_SPEED_MAP(125_KHZ, 125K);
            COINES_NRF_PLATFORM_SPEED_MAP(250_KHZ, 250K);
            COINES_NRF_PLATFORM_SPEED_MAP(500_KHZ, 500K);
            COINES_NRF_PLATFORM_SPEED_MAP(1_MHZ, 1M);
            COINES_NRF_PLATFORM_SPEED_MAP(2_MHZ, 2M);
            COINES_NRF_PLATFORM_SPEED_MAP(4_MHZ, 4M);
            COINES_NRF_PLATFORM_SPEED_MAP(8_MHZ, 8M);

                default:
                    coines_spi_intf[bus].config.frequency = NRF_SPIM_FREQ_2M;
            }
            if (NRFX_SUCCESS == nrfx_spim_init(&coines_spi_intf[bus].peripheral_instance, &coines_spi_intf[bus].config, NULL, NULL))
            {
                coines_spi_intf[bus].enabled = true; /* Set SPI bus enabled */

                /*Modifying the drive modes and pull configurations*/
                nrf_gpio_cfg(coines_spi_intf[bus].config.sck_pin,
                             NRF_GPIO_PIN_DIR_OUTPUT,
                             NRF_GPIO_PIN_INPUT_DISCONNECT,
                             NRF_GPIO_PIN_NOPULL,
                             NRF_GPIO_PIN_H0H1,
                             NRF_GPIO_PIN_NOSENSE);

                nrf_gpio_cfg(coines_spi_intf[bus].config.mosi_pin,
                             NRF_GPIO_PIN_DIR_OUTPUT,
                             NRF_GPIO_PIN_INPUT_DISCONNECT,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_H0H1,
                             NRF_GPIO_PIN_NOSENSE);

                nrf_gpio_cfg(coines_spi_intf[bus].config.miso_pin,
                             NRF_GPIO_PIN_DIR_INPUT,
                             NRF_GPIO_PIN_INPUT_CONNECT,
                             NRF_GPIO_PIN_PULLDOWN,
                             NRF_GPIO_PIN_H0H1,
                             NRF_GPIO_PIN_NOSENSE);

                /* Set the SPI instance status to enabled */
                if (COINES_SUCCESS != coines_set_spi_instance(bus, COINES_ENABLE))
                {
                    retval = COINES_E_SPI_CONFIG_FAILED;
                }
            }
            else
            {
                coines_spi_intf[bus].enabled = false;    /* Set SPI bus status to disabled */
                retval = COINES_E_COMM_INIT_FAILED;

                /* Set the SPI instance status to disabled */
                if (COINES_SUCCESS != coines_set_spi_instance(bus, COINES_DISABLE))
                {
                    retval = COINES_E_SPI_CONFIG_FAILED;
                }
            }
        }
        else
        {
            retval = COINES_E_SPI_CONFIG_EXIST;
        }
    }
    else
    {
        retval = COINES_E_SPI_INVALID_BUS_INTF;
    }

    return retval;
}

/*!
 *  @brief This API is used to de-configure the SPI bus
 */
int16_t coines_deconfig_spi_bus(enum coines_spi_bus bus)
{
    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {   
        if (coines_is_spi_enabled(bus))
        {
            nrfx_spim_uninit(&coines_spi_intf[bus].peripheral_instance);
            coines_spi_intf[bus].enabled = false;

            /* Set the SPI instance status to disabled */
            if (COINES_SUCCESS != coines_set_spi_instance(bus, COINES_DISABLE))
            {
                return COINES_E_SPI_CONFIG_FAILED;
            }
        }
        else
        {
            return COINES_E_SPI_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_SPI_INVALID_BUS_INTF;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the spi bus with 8 bit or 16 bit length
 *
 */
int16_t coines_config_word_spi_bus(enum coines_spi_bus bus,
                                   enum coines_spi_speed spi_speed,
                                   enum coines_spi_mode spi_mode,
                                   enum coines_spi_transfer_bits spi_transfer_bits)
{
   /*
   Nordic's SPI initialization function is the same for both 8-bit and 16-bit transfers. 
   */
  (void)spi_transfer_bits;
   return coines_config_spi_bus(bus, spi_speed, spi_mode); 
}

/*!
 *  @brief This API is used to map the I2C pins with respective I2C bus interfaces
 */
static void coines_get_i2c_pin_map(enum coines_i2c_bus bus, enum coines_i2c_pin_map pin_map)
{
    switch (pin_map)
    {
        case COINES_I2C_PIN_PRIMARY:
            coines_i2c_intf[bus].config.sda = I2C0_SEN_SDA_PIN;
            coines_i2c_intf[bus].config.scl = I2C0_SEN_SCL_PIN;
            break;

        case COINES_I2C_PIN_SECONDARY:
            coines_i2c_intf[bus].config.sda = I2C1_SEN_SDA_PIN;
            coines_i2c_intf[bus].config.scl = I2C1_SEN_SCL_PIN;
            break;

        case COINES_I2C_PIN_INTERNAL:
            coines_i2c_intf[bus].config.sda = I2C_INTERNAL_SDA_PIN;
            coines_i2c_intf[bus].config.scl = I2C_INTERNAL_SCL_PIN;
            break;


        case COINES_I2C_PIN_DEFAULT:
            break;

        default:
            break;
    }
}

/*!
 *  @brief This internal API is used to configure the I2C bus with explicit pin mapping
 */
int16_t coines_config_i2c_bus_internal(enum coines_i2c_bus bus,
                                       enum coines_i2c_mode i2c_mode,
                                       enum coines_i2c_pin_map pin_map)
{
    int16_t retval = COINES_SUCCESS;

    if (COINES_SUCCESS != coines_get_i2c_instance(bus))
    {
        return COINES_E_I2C_CONFIG_FAILED;
    }


    if (COINES_I2C_PIN_DEFAULT != pin_map)
    {
        coines_get_i2c_pin_map(bus, pin_map);
    }

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {

        if (!coines_is_i2c_enabled(bus)) /* Check whether I2C bus is already enabled */
        {

            nrf_gpio_cfg(coines_i2c_intf[bus].config.sda,
                         NRF_GPIO_PIN_DIR_INPUT,
                         NRF_GPIO_PIN_INPUT_CONNECT,
                         NRF_GPIO_PIN_PULLUP,
                         NRF_GPIO_PIN_H0D1,
                         NRF_GPIO_PIN_NOSENSE);

            nrf_gpio_cfg(coines_i2c_intf[bus].config.scl,
                         NRF_GPIO_PIN_DIR_INPUT,
                         NRF_GPIO_PIN_INPUT_CONNECT,
                         NRF_GPIO_PIN_PULLUP,
                         NRF_GPIO_PIN_H0D1,
                         NRF_GPIO_PIN_NOSENSE);

            if (nrfx_twim_init(&coines_i2c_intf[bus].peripheral_instance, &coines_i2c_intf[bus].config, coines_i2c_intf[bus].event_handler,
                               NULL) == NRFX_SUCCESS)
            {   

                nrfx_twim_enable(&coines_i2c_intf[bus].peripheral_instance);

                if (i2c_mode == COINES_I2C_STANDARD_MODE)
                {
                    coines_i2c_intf[bus].config.frequency = NRF_TWIM_FREQ_100K;
                    nrf_twim_frequency_set(coines_i2c_intf[bus].peripheral_instance.p_twim, NRF_TWIM_FREQ_100K);
                }
                else
                {
                    coines_i2c_intf[bus].config.frequency = NRF_TWIM_FREQ_400K;
                    nrf_twim_frequency_set(coines_i2c_intf[bus].peripheral_instance.p_twim, NRF_TWIM_FREQ_400K);
                }

                coines_i2c_intf[bus].enabled = true; /* Set I2C bus status to enabled */

                /* Set the I2C instance status to enabled */
                if (COINES_SUCCESS != coines_set_i2c_instance(bus, COINES_ENABLE))
                {
                    retval = COINES_E_I2C_CONFIG_FAILED;
                }
            }
            else
            {
                coines_i2c_intf[bus].enabled = false;    /* Set I2C bus status to disabled */
                retval = COINES_E_COMM_INIT_FAILED;

                /* Set the I2C instance status to disabled */
                if (COINES_SUCCESS != coines_set_i2c_instance(bus, COINES_DISABLE))
                {
                    retval = COINES_E_I2C_CONFIG_FAILED;
                }
            }
        }
        else
        {
            retval = COINES_E_I2C_CONFIG_EXIST;
        }
    }
    else
    {
        retval = COINES_E_I2C_INVALID_BUS_INTF;
    }

    return retval;
}

/*!
 *  @brief This API is used to configure the I2C bus
 */
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode)
{
    return coines_config_i2c_bus_internal(bus, i2c_mode, COINES_I2C_PIN_DEFAULT);
}

/*!
 *  @brief This API is used to de-configure the I2C bus
 */
int16_t coines_deconfig_i2c_bus(enum coines_i2c_bus bus)
{   
    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus)) /* Check whether I2C bus is enabled */
        {
            nrfx_twim_disable(&coines_i2c_intf[bus].peripheral_instance);
            nrfx_twim_uninit(&coines_i2c_intf[bus].peripheral_instance);

            coines_i2c_intf[bus].enabled = false;    /*  Set I2C bus status to disabled */

            /* Set the I2C instance status to disabled */
            if (COINES_SUCCESS != coines_set_i2c_instance(bus, COINES_DISABLE))
            {
                return COINES_E_I2C_CONFIG_FAILED;
            }
        }
        else
        {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 */
static int8_t i2c_write(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint16_t count , enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    nrfx_err_t error;

    uint8_t reg_addr_buffer[REG_ADDR_BUFFER_SIZE] = {0};
    uint8_t reg_addr_length = 0;
    uint8_t buffer[count + 2];

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            reg_addr_length = prepare_reg_addr_buffer(reg_addr_buffer, reg_addr, i2c_transfer_bits);
            memcpy(&buffer[0], reg_addr_buffer, reg_addr_length);
            memcpy(&buffer[reg_addr_length], reg_data, count);

            /*lint -e785*/
            nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(dev_addr, buffer, (count + reg_addr_length));
            /*lint +e785*/

            if (bus == COINES_I2C_BUS_1)
            {
                while (nrfx_twim_is_busy(&coines_i2c_intf[bus].peripheral_instance))
                    ;
            }

            coines_i2c_intf[bus].txrx_status = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_intf[bus].peripheral_instance, &write_desc, NRFX_TWIM_FLAG_TX_POSTINC);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_intf[bus].txrx_status == COINES_I2C_TX_NONE))
            {
                coines_yield();
            }

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_intf[bus].txrx_status != COINES_I2C_TX_SUCCESS)
            {
                coines_i2c_bus_recover(bus);

                return COINES_E_COMM_IO_ERROR;
            }

            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 */
int8_t coines_write_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    return i2c_write(bus, dev_addr, (uint16_t)reg_addr, reg_data, count, COINES_I2C_TRANSFER_8BIT);
}

/*!
 *  @brief This API is used to write 16-bit register data on the I2C device.
 */
int8_t coines_write_16bit_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    uint16_t swapped_reg_data[count];

    if(i2c_transfer_bits == COINES_I2C_TRANSFER_16BIT)
    {
        /* Conversion of payload from big endian to little endian */
        swap_endianness(swapped_reg_data, (uint16_t*)reg_data, count);

        return i2c_write(bus, dev_addr, reg_addr, (uint8_t*)swapped_reg_data, (uint16_t)(count * sizeof(uint16_t)), COINES_I2C_TRANSFER_16BIT);
    }
    else
    {
        return i2c_write(bus, dev_addr, reg_addr, (uint8_t*)reg_data, count, COINES_I2C_TRANSFER_16BIT);
    }
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
static int8_t i2c_read(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, uint8_t *reg_data, uint16_t count, enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    nrfx_err_t error;

    uint8_t reg_addr_buffer[REG_ADDR_BUFFER_SIZE] = {0};
    uint8_t reg_addr_length = 0;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            reg_addr_length = prepare_reg_addr_buffer(reg_addr_buffer, reg_addr, i2c_transfer_bits);
            nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_TXRX(dev_addr, reg_addr_buffer, reg_addr_length, reg_data, count);

            coines_i2c_intf[bus].txrx_status = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_intf[bus].peripheral_instance,
                                   &read_desc,
                                   NRFX_TWIM_FLAG_RX_POSTINC | NRFX_TWIM_FLAG_REPEATED_XFER);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_intf[bus].txrx_status == COINES_I2C_TX_NONE))
            {
                coines_yield();
            }

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_intf[bus].txrx_status != COINES_I2C_TX_SUCCESS)
            {
                coines_i2c_bus_recover(bus);

                return COINES_E_COMM_IO_ERROR;
            }

            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
int8_t coines_read_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    return i2c_read(bus, dev_addr, (uint16_t)reg_addr, reg_data, count, COINES_I2C_TRANSFER_8BIT);
}

/*!
 *  @brief This API is used to read 16-bit register data from the I2C device.
 */
int8_t coines_read_16bit_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_i2c_transfer_bits i2c_transfer_bits)
{
    int8_t ret;
    
    if(i2c_transfer_bits == COINES_I2C_TRANSFER_16BIT)
    {
        ret = i2c_read(bus, dev_addr, reg_addr, (uint8_t*)reg_data, (uint16_t)(count * sizeof(uint16_t)), COINES_I2C_TRANSFER_16BIT);
        /* Conversion of payload from big endian to little endian */
        swap_endianness((uint16_t*)reg_data, (uint16_t*)reg_data, count);
    }
    else
    {
        ret = i2c_read(bus, dev_addr, reg_addr, (uint8_t*)reg_data, count, COINES_I2C_TRANSFER_16BIT);
    }

    return ret;
}

int8_t coines_i2c_set(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count)
{
    nrfx_err_t error;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            /*lint -e785*/
            nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(dev_addr, data, count);
            /*lint +e785*/
            coines_i2c_intf[bus].txrx_status = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_intf[bus].peripheral_instance, &write_desc, NRFX_TWIM_FLAG_TX_POSTINC);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_intf[bus].txrx_status == COINES_I2C_TX_NONE))
                ;

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_intf[bus].txrx_status != COINES_I2C_TX_SUCCESS)
            {
                coines_i2c_bus_recover(bus);
            }

            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

int8_t coines_i2c_get(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count)
{
    nrfx_err_t error;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            /*lint -e785*/
            nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_RX(dev_addr, data, count);
            /*lint +e785*/

            coines_i2c_intf[bus].txrx_status = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_intf[bus].peripheral_instance, &read_desc, NRFX_TWIM_FLAG_RX_POSTINC);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_intf[bus].txrx_status == COINES_I2C_TX_NONE))
                ;

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_intf[bus].txrx_status != COINES_I2C_TX_SUCCESS)
            {
                coines_i2c_bus_recover(bus);
            }

            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_I2C_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_I2C_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 */
static int8_t spi_write(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, uint8_t *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    nrfx_err_t error;
    uint8_t reg_addr_buffer[REG_ADDR_BUFFER_SIZE] = {0};
    uint8_t reg_addr_length = 0;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (coines_is_spi_enabled(bus))
        {
            uint32_t pin_no = multi_io_map[cs_pin];
            if (pin_no == 0 || pin_no == 0xff)
            {
                return COINES_E_FAILURE;
            }
            else
            {
                nrf_gpio_cfg_output(pin_no);
            }

            /* Activate CS pin */
            nrf_gpio_pin_write(pin_no, 0);

            reg_addr_length = prepare_reg_addr_buffer(reg_addr_buffer, reg_addr, spi_transfer_bits);

            /*lint -e789 */
            coines_spi_intf[bus].txrx_desc.p_tx_buffer = reg_addr_buffer;
            coines_spi_intf[bus].txrx_desc.tx_length = reg_addr_length;
            coines_spi_intf[bus].txrx_desc.p_rx_buffer = NULL;
            coines_spi_intf[bus].txrx_desc.rx_length = 0;

            error = nrfx_spim_xfer(&coines_spi_intf[bus].peripheral_instance, &coines_spi_intf[bus].txrx_desc, 0);

            if (error == NRFX_SUCCESS)
            {
                coines_spi_intf[bus].txrx_desc.p_tx_buffer = reg_data;
                coines_spi_intf[bus].txrx_desc.tx_length = count;
                coines_spi_intf[bus].txrx_desc.p_rx_buffer = NULL;
                coines_spi_intf[bus].txrx_desc.rx_length = 0;

                error = nrfx_spim_xfer(&coines_spi_intf[bus].peripheral_instance, &coines_spi_intf[bus].txrx_desc, 0);

                if (error == NRFX_ERROR_INVALID_ADDR)
                {
                    /* The transfer didn't occur because the
                    * SPI peripheral requires that the buffer
                    * is in the RAM Data section
                    */
                    uint8_t temp_buff[count];
                    memcpy(temp_buff, reg_data, count);
                    coines_spi_intf[bus].txrx_desc.p_tx_buffer = temp_buff;

                    error = nrfx_spim_xfer(&coines_spi_intf[bus].peripheral_instance, &coines_spi_intf[bus].txrx_desc, 0);
                }
            }

            /* Deactivate CS pin */
            nrf_gpio_pin_write(pin_no, 1);

            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_SPI_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_SPI_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 */
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    return spi_write(bus, cs_pin, (uint16_t)reg_addr, reg_data, count, COINES_SPI_TRANSFER_8BIT);
}

/*!
 *  @brief This API is used to write 16-bit register data on the SPI device.
 */
int8_t coines_write_16bit_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    uint16_t swapped_reg_data[count];

    if(spi_transfer_bits == COINES_SPI_TRANSFER_16BIT)
    {
        /* Conversion of payload from big endian to little endian */
        swap_endianness(swapped_reg_data, (uint16_t*)reg_data, count);

        return spi_write(bus, cs_pin, reg_addr, (uint8_t*)swapped_reg_data, (uint16_t)(count * sizeof(uint16_t)), COINES_SPI_TRANSFER_16BIT);
    }
    else
    {
        return spi_write(bus, cs_pin, reg_addr, (uint8_t*)reg_data, count, COINES_SPI_TRANSFER_16BIT);
    }
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
static int8_t spi_read(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, uint8_t *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    nrfx_err_t error;
    uint8_t reg_addr_buffer[REG_ADDR_BUFFER_SIZE] = {0};
    uint8_t reg_addr_length = 0;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (coines_is_spi_enabled(bus))
        {
            uint32_t pin_num = multi_io_map[cs_pin];
            if (pin_num == 0 || pin_num == 0xff)
            {
                return COINES_E_FAILURE;
            }
            else
            {
                nrf_gpio_cfg_output(pin_num);
            }

            /* Activate CS pin */
            nrf_gpio_pin_write(pin_num, 0);

            reg_addr_length = prepare_reg_addr_buffer(reg_addr_buffer, reg_addr, spi_transfer_bits);

            coines_spi_intf[bus].txrx_desc.p_tx_buffer = reg_addr_buffer;
            coines_spi_intf[bus].txrx_desc.tx_length = reg_addr_length;
            coines_spi_intf[bus].txrx_desc.p_rx_buffer = NULL;
            coines_spi_intf[bus].txrx_desc.rx_length = 0;

            error = nrfx_spim_xfer(&coines_spi_intf[bus].peripheral_instance, &coines_spi_intf[bus].txrx_desc, 0);

            if (error == NRFX_SUCCESS)
            {
                coines_spi_intf[bus].txrx_desc.p_tx_buffer = NULL;
                coines_spi_intf[bus].txrx_desc.tx_length = 0;
                coines_spi_intf[bus].txrx_desc.p_rx_buffer = reg_data;
                coines_spi_intf[bus].txrx_desc.rx_length = count;

                error = nrfx_spim_xfer(&coines_spi_intf[bus].peripheral_instance, &coines_spi_intf[bus].txrx_desc, 0);
            }

            /* Deactivate CS pin */
            nrf_gpio_pin_write(pin_num, 1);

            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_SPI_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_SPI_INVALID_BUS_INTF;
    }
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_read_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    return spi_read(bus, cs_pin, (uint16_t)reg_addr, reg_data, count, COINES_SPI_TRANSFER_8BIT);
}

/*!
 *  @brief This API is used to read 16-bit register data from the SPI device.
 */
int8_t coines_read_16bit_spi(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    int8_t ret;
    
    if(spi_transfer_bits == COINES_SPI_TRANSFER_16BIT)
    {
        ret = spi_read(bus, cs_pin, reg_addr, (uint8_t*)reg_data,(uint16_t)(count * sizeof(uint16_t)), COINES_SPI_TRANSFER_16BIT);
        /* Conversion of payload from big endian to little endian */
        swap_endianness((uint16_t*)reg_data, (uint16_t*)reg_data, count);
    }
    else
    {
        ret = spi_read(bus, cs_pin, reg_addr, (uint8_t*)reg_data, count, COINES_SPI_TRANSFER_16BIT);
    }

    return ret;
}

/*!
 *  @brief This API is used to initialize the UART communication
 */
int8_t coines_uart_init(enum coines_uart_instance uart_instance,
                        enum coines_uart_parity parity,
                        enum coines_uart_flow_control flow_control,
                        uint32_t baud_rate)
{
    uint32_t error;

    if(uart_instance != COINES_UART_0)
    {
        return COINES_E_UART_INSTANCE_NOT_SUPPORT;
    }
    error = uart_init(parity, flow_control, baud_rate);

    if (error != NRF_SUCCESS)
    {
        return COINES_E_UART_INIT_FAILED;
    }

    return COINES_SUCCESS;

}

/*!
 *  @brief This API is used to read the data in UART communication
 */
uint16_t coines_uart_read(enum coines_uart_instance uart_instance, uint8_t *buffer, uint16_t length)
{
    uint16_t bytes_read;

    if(uart_instance != COINES_UART_0)
    {
        return 0U;
    }

    bytes_read = uart_read(buffer, length);

    return bytes_read;

}

/*!
 *  @brief This API is used to write the data in UART communication
 */
int8_t coines_uart_write(enum coines_uart_instance uart_instance, uint8_t *buffer, uint16_t length)
{
    uint32_t ret_value;

    if(uart_instance != COINES_UART_0)
    {
        return COINES_E_UART_INSTANCE_NOT_SUPPORT;
    }

    ret_value = uart_write(buffer, length);

    if (ret_value != NRF_SUCCESS)
    {
        return COINES_E_UART_WRITE_FAILED;
    }

    return COINES_SUCCESS;
}

/**
 * @brief This API is used to perform a SPI transfer.
 */
int8_t coines_spi_transfer(enum coines_spi_bus bus, uint8_t cs_pin, uint8_t *tx_buff, uint32_t tx_len, uint8_t *rx_buff, uint32_t rx_len)
{
    nrfx_err_t error;

    if ((tx_buff == NULL && tx_len > 0) ||
        (rx_buff == NULL && rx_len > 0))
    {
        return COINES_E_INVALID_PARAM;
    }

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (coines_is_spi_enabled(bus))
        {
            uint32_t pin_no = multi_io_map[cs_pin];
            if (pin_no == 0 || pin_no == 0xff)
            {
                return COINES_E_FAILURE;
            }
            else
            {
                nrf_gpio_cfg_output(pin_no);
            }

            /* Activate CS pin */
            nrf_gpio_pin_write(pin_no, 0);

            nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_buff, tx_len, rx_buff, rx_len);

            /* Perform SPI transaction */
            error = nrfx_spim_xfer(&coines_spi_intf[bus].peripheral_instance, &xfer_desc, 0);


            /* Deactivate CS pin */
            nrf_gpio_pin_write(pin_no, 1);

            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        else
        {
            return COINES_E_SPI_BUS_NOT_ENABLED;
        }
    }
    else
    {
        return COINES_E_SPI_INVALID_BUS_INTF;
    }
}

/** @brief Callback function invoked when an I2C bus transaction is completed. */
__attribute__((weak)) void coines_i2c_bus_transaction_complete_callback(enum coines_i2c_bus bus, bool status)
{
    ;
}