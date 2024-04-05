/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    mcu_app3x_support.c
 * @date    May 25, 2021
 * @brief   COINES_SDK support file for mcu_app30.c
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "mcu_app3x_support.h"
#include "mcu_app3x_interface.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
/* Indicates if operation on I2C has completed. */
static volatile uint8_t coines_i2c_txrx_status[COINES_I2C_BUS_MAX] = { COINES_I2C_TX_FAILED, COINES_I2C_TX_FAILED };

static const nrfx_twim_t coines_i2c_instance[COINES_I2C_BUS_MAX] =
{ NRFX_TWIM_INSTANCE(TWIM0_INSTANCE), NRFX_TWIM_INSTANCE(TWIM1_INSTANCE) };

static nrfx_twim_config_t coines_i2c_config[COINES_I2C_BUS_MAX] = {
    {                                                  /* Primary sensor interface */
        .scl = I2C0_SEN_SCL_PIN, .sda = I2C0_SEN_SDA_PIN, .frequency = NRF_TWIM_FREQ_400K,
        .interrupt_priority = (uint8_t)APP_IRQ_PRIORITY_HIGH, .hold_bus_uninit = false
    }, { /* Temperature (Adapter) interface */
#if defined(MCU_APP30)
        .scl = I2C1_EXTERNAL_TEMP_SCL_PIN, .sda = I2C1_EXTERNAL_TEMP_SDA_PIN,
#else
        .scl = I2C1_INTERNAL_TEMP_SCL_PIN, .sda = I2C1_INTERNAL_TEMP_SDA_PIN,
#endif
        .frequency = NRF_TWIM_FREQ_400K, .interrupt_priority = (uint8_t)APP_IRQ_PRIORITY_HIGH, .hold_bus_uninit = false
    }

};

typedef void (*coines_i2c_callback)(nrfx_twim_evt_t const *, void *);

static void coines_i2c0_event_handler(nrfx_twim_evt_t const *p_event, void *p_context);
static void coines_i2c1_event_handler(nrfx_twim_evt_t const *p_event, void *p_context);

static coines_i2c_callback coines_i2c_event_handler[COINES_I2C_BUS_MAX] =
{ coines_i2c0_event_handler, coines_i2c1_event_handler };

static int16_t coines_i2c_bus_recover(enum coines_i2c_bus bus);

static void coines_get_i2c_pin_map(enum coines_i2c_bus bus, enum coines_i2c_pin_map pin_map);
static int16_t coines_set_spi_instance(enum coines_spi_bus bus, uint8_t enable);
static int16_t coines_set_i2c_instance(enum coines_i2c_bus bus, uint8_t enable);
static bool coines_is_spi_enabled(enum coines_spi_bus bus);

static volatile bool is_i2c_enabled[COINES_I2C_BUS_MAX] = { false, false };

static const nrfx_spim_t coines_spi_instance[COINES_SPI_BUS_MAX] =
{ NRFX_SPIM_INSTANCE(SPIM0_INSTANCE), NRFX_SPIM_INSTANCE(SPIM1_INSTANCE) };

static volatile bool is_spi_enabled[COINES_SPI_BUS_MAX] = { false, false };

static nrfx_spim_config_t coines_spi_config[COINES_SPI_BUS_MAX] = {
    {              /* Primary sensor interface */
        .sck_pin = SPI0_SEN_SCK_PIN, .mosi_pin = SPI0_SEN_MOSI_PIN, .miso_pin = SPI0_SEN_MISO_PIN,
        .ss_pin = NRFX_SPIM_PIN_NOT_USED, .ss_active_high = false,
        .irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY, .orc = 0xFF, .frequency = NRF_SPIM_FREQ_4M,
        .mode = NRF_SPIM_MODE_0, .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST, NRFX_SPIM_DEFAULT_EXTENDED_CONFIG
    }, {           /* OIS controller interface */
        .sck_pin = SPI1_OIS_SCK_PIN, .mosi_pin = SPI1_OIS_MOSI_PIN, .miso_pin = SPI1_OIS_MISO_PIN,
        .ss_pin = NRFX_SPIM_PIN_NOT_USED, .ss_active_high = false,
        .irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY, .orc = 0xFF, .frequency = NRF_SPIM_FREQ_4M,
        .mode = NRF_SPIM_MODE_0, .bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST, NRFX_SPIM_DEFAULT_EXTENDED_CONFIG
    },
};

static nrfx_spim_xfer_desc_t coines_spi_txrx_desc[COINES_SPI_BUS_MAX]; /**< SPI transfer descriptor. */

/* idx = 0 ==> Primary sensor I2C/SPI interface; User configurable */

/* idx = 1 ==> Ble temperature I2C interface; Enabled internally but
 *              can be reconfigured to other sensors if requested
 *              e.g. adapter board, since the on-board (BLE)
 *              temperature sensor has low priority */

/* idx = 2 ==> Secondary sensor SPI interface; User configurable */
/* idx = 3 ==> External flash SPI interface; By default enabled */
static uint8_t coines_spi_twi_instances[4] = { 0, 0, 0, 1 };

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/

/*!
 * @brief   This function manages the I2C0 event call back
 */
static void coines_i2c0_event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            coines_i2c_txrx_status[COINES_I2C_BUS_0] = COINES_I2C_TX_SUCCESS;
            break;
        default:
            coines_i2c_txrx_status[COINES_I2C_BUS_0] = COINES_I2C_TX_FAILED;
            break;
    }
}

/*!
 * @brief   This function manages the I2C1 event call back
 */
static void coines_i2c1_event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            coines_i2c_txrx_status[COINES_I2C_BUS_1] = COINES_I2C_TX_SUCCESS;
            break;
        default:
            coines_i2c_txrx_status[COINES_I2C_BUS_1] = COINES_I2C_TX_FAILED;
            break;
    }
}

/*!
 * @brief   This function returns the I2C bus enabled status
 */
bool coines_is_i2c_enabled(enum coines_i2c_bus bus)
{
    return is_i2c_enabled[bus] ? true : false;
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
            nrfx_twim_uninit(&coines_i2c_instance[bus]);
            is_i2c_enabled[bus] = false; /* Set I2C bus status to disabled */

            /* TODO: Re-initialize originally set I2C speed ! */
            error = nrfx_twim_init(&coines_i2c_instance[bus],
                                   &coines_i2c_config[bus],
                                   coines_i2c_event_handler[bus],
                                   NULL);
            nrfx_twim_enable(&coines_i2c_instance[bus]);
            if (NRFX_SUCCESS != error)
            {
                printf("I2C recovery failed\n\r");
                result = COINES_E_FAILURE;
            }
            else
            {
                is_i2c_enabled[bus] = true; /* Enabled I2C bus after recovery */
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
static bool coines_is_spi_enabled(enum coines_spi_bus bus)
{
    return is_spi_enabled[bus] ? true : false;
}

/*!
 * @brief   This function returns the I2C instance status
 */
static int16_t coines_get_i2c_instance(enum coines_i2c_bus bus)
{
    int16_t return_val = COINES_SUCCESS;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        return_val = coines_spi_twi_instances[bus];
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

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (COINES_ENABLE == enable)
        {
            if (0 != coines_spi_twi_instances[bus])
            {
                return COINES_E_FAILURE;
            }

            coines_spi_twi_instances[bus] = COINES_ENABLE;
        }
        else if (COINES_DISABLE == enable)
        {
            coines_spi_twi_instances[bus] = COINES_DISABLE;
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
static int16_t coines_get_spi_instance(enum coines_spi_bus bus)
{
    int16_t return_val = COINES_SUCCESS;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (bus == COINES_SPI_BUS_1)
        {
            bus = COINES_SPI_BUS_MAX;
        }

        return_val = coines_spi_twi_instances[bus];
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
static int16_t coines_set_spi_instance(enum coines_spi_bus bus, uint8_t enable)
{
    int16_t return_val = COINES_SUCCESS;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        /* Instance 1 is by default used as I2C for BLE temperature
        * so SPI_BUS_1 has to use instance 2 of NRF peripherals
        */
        if (bus == COINES_SPI_BUS_1)
        {
            bus = COINES_SPI_BUS_MAX;
        }

        if (COINES_ENABLE == enable)
        {
            if (0 != coines_spi_twi_instances[bus])
            {
                return COINES_E_FAILURE;
            }

            coines_spi_twi_instances[bus] = COINES_ENABLE;
        }
        else if (COINES_DISABLE == enable)
        {
            coines_spi_twi_instances[bus] = COINES_DISABLE;
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
            coines_spi_config[bus].mode = (nrf_spim_mode_t)spi_mode;

#define COINES_NRF_SPEED_MAP(coines_spi, nrf_spi)  \
        case  COINES_SPI_SPEED_##coines_spi:         \
            coines_spi_config[bus].frequency = NRF_SPIM_FREQ_##nrf_spi; \
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

            COINES_NRF_SPEED_MAP(10_MHZ, 8M);

                default:
                    coines_spi_config[bus].frequency = NRF_SPIM_FREQ_2M;
            }
            if (NRFX_SUCCESS == nrfx_spim_init(&coines_spi_instance[bus], &coines_spi_config[bus], NULL, NULL))
            {
                is_spi_enabled[bus] = true; /* Set SPI bus enabled */

                /*Modifying the drive modes and pull configurations*/
                nrf_gpio_cfg(coines_spi_config[bus].sck_pin,
                             NRF_GPIO_PIN_DIR_OUTPUT,
                             NRF_GPIO_PIN_INPUT_DISCONNECT,
                             NRF_GPIO_PIN_NOPULL,
                             NRF_GPIO_PIN_H0H1,
                             NRF_GPIO_PIN_NOSENSE);

                nrf_gpio_cfg(coines_spi_config[bus].mosi_pin,
                             NRF_GPIO_PIN_DIR_OUTPUT,
                             NRF_GPIO_PIN_INPUT_DISCONNECT,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_H0H1,
                             NRF_GPIO_PIN_NOSENSE);

                nrf_gpio_cfg(coines_spi_config[bus].miso_pin,
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
                is_spi_enabled[bus] = false;    /* Set SPI bus status to disabled */
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
            nrfx_spim_uninit(&coines_spi_instance[bus]);
            is_spi_enabled[bus] = false;

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
 *  @brief This API is used to map the I2C pins with respective I2C bus interfaces
 */
static void coines_get_i2c_pin_map(enum coines_i2c_bus bus, enum coines_i2c_pin_map pin_map)
{
    switch (pin_map)
    {
        case COINES_I2C_PIN_PRIMARY:
            coines_i2c_config[bus].sda = I2C0_SEN_SDA_PIN;
            coines_i2c_config[bus].scl = I2C0_SEN_SCL_PIN;
            break;

        case COINES_I2C_PIN_INTERNAL_TEMP:
            coines_i2c_config[bus].sda = I2C1_INTERNAL_TEMP_SDA_PIN;
            coines_i2c_config[bus].scl = I2C1_INTERNAL_TEMP_SCL_PIN;
            break;

        case COINES_I2C_PIN_SECONDARY:
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

            nrf_gpio_cfg(coines_i2c_config[bus].sda,
                         NRF_GPIO_PIN_DIR_INPUT,
                         NRF_GPIO_PIN_INPUT_CONNECT,
                         NRF_GPIO_PIN_PULLUP,
                         NRF_GPIO_PIN_H0D1,
                         NRF_GPIO_PIN_NOSENSE);

            nrf_gpio_cfg(coines_i2c_config[bus].scl,
                         NRF_GPIO_PIN_DIR_INPUT,
                         NRF_GPIO_PIN_INPUT_CONNECT,
                         NRF_GPIO_PIN_PULLUP,
                         NRF_GPIO_PIN_H0D1,
                         NRF_GPIO_PIN_NOSENSE);

            if (nrfx_twim_init(&coines_i2c_instance[bus], &coines_i2c_config[bus], coines_i2c_event_handler[bus],
                               NULL) == NRFX_SUCCESS)
            {

                nrfx_twim_enable(&coines_i2c_instance[bus]);

                if (i2c_mode == COINES_I2C_STANDARD_MODE)
                {
                    coines_i2c_config[bus].frequency = NRF_TWIM_FREQ_100K;
                    nrf_twim_frequency_set(coines_i2c_instance[bus].p_twim, NRF_TWIM_FREQ_100K);
                }
                else
                {
                    coines_i2c_config[bus].frequency = NRF_TWIM_FREQ_400K;
                    nrf_twim_frequency_set(coines_i2c_instance[bus].p_twim, NRF_TWIM_FREQ_400K);
                }

                is_i2c_enabled[bus] = true; /* Set I2C bus status to enabled */

                /* Set the I2C instance status to enabled */
                if (COINES_SUCCESS != coines_set_i2c_instance(bus, COINES_ENABLE))
                {
                    retval = COINES_E_I2C_CONFIG_FAILED;
                }
            }
            else
            {
                is_i2c_enabled[bus] = false;    /* Set I2C bus status to disabled */
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
            nrfx_twim_disable(&coines_i2c_instance[bus]);
            nrfx_twim_uninit(&coines_i2c_instance[bus]);

            is_i2c_enabled[bus] = false;    /*  Set I2C bus status to disabled */

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
int8_t coines_write_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    nrfx_err_t error;

    uint8_t buffer[count + 1];

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            buffer[0] = reg_addr;
            memcpy(&buffer[1], reg_data, count);

            nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(dev_addr, buffer, (count + 1));

            if (bus == COINES_I2C_BUS_1)
            {
                while (nrfx_twim_is_busy(&coines_i2c_instance[bus]))
                    ;
            }

            coines_i2c_txrx_status[bus] = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_instance[bus], &write_desc, NRFX_TWIM_FLAG_TX_POSTINC);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_txrx_status[bus] == COINES_I2C_TX_NONE))
            {
                coines_yield();
            }

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_txrx_status[bus] != COINES_I2C_TX_SUCCESS)
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
    nrfx_err_t error;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_TXRX(dev_addr, &reg_addr, 1, reg_data, count);

            coines_i2c_txrx_status[bus] = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_instance[bus],
                                   &read_desc,
                                   NRFX_TWIM_FLAG_RX_POSTINC | NRFX_TWIM_FLAG_REPEATED_XFER);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_txrx_status[bus] == COINES_I2C_TX_NONE))
            {
                coines_yield();
            }

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_txrx_status[bus] != COINES_I2C_TX_SUCCESS)
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

int8_t coines_i2c_set(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count)
{
    nrfx_err_t error;

    if ((bus < COINES_I2C_BUS_MAX) && (bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(bus))
        {
            nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(dev_addr, data, count);

            coines_i2c_txrx_status[bus] = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_instance[bus], &write_desc, NRFX_TWIM_FLAG_TX_POSTINC);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_txrx_status[bus] == COINES_I2C_TX_NONE))
                ;

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_txrx_status[bus] != COINES_I2C_TX_SUCCESS)
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
            nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_RX(dev_addr, data, count);

            coines_i2c_txrx_status[bus] = COINES_I2C_TX_NONE;
            error = nrfx_twim_xfer(&coines_i2c_instance[bus], &read_desc, NRFX_TWIM_FLAG_RX_POSTINC);

            /* Timeout the I2C operation after 1000 ms */
            volatile uint32_t t = coines_get_millis();
            while ((coines_get_millis() - t < I2C_TIMEOUT_MS) && (coines_i2c_txrx_status[bus] == COINES_I2C_TX_NONE))
                ;

            /* If I2C transfer has timed out, recover the I2C bus */
            if (coines_i2c_txrx_status[bus] != COINES_I2C_TX_SUCCESS)
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
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    nrfx_err_t error;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (coines_is_spi_enabled(bus))
        {
            uint32_t pin_no = multi_io_map[dev_addr];
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

            /*lint -e789 */
            coines_spi_txrx_desc[bus].p_tx_buffer = &reg_addr;
            coines_spi_txrx_desc[bus].tx_length = 1;
            coines_spi_txrx_desc[bus].p_rx_buffer = NULL;
            coines_spi_txrx_desc[bus].rx_length = 0;

            error = nrfx_spim_xfer(&coines_spi_instance[bus], &coines_spi_txrx_desc[bus], 0);

            if (error == NRFX_SUCCESS)
            {
                coines_spi_txrx_desc[bus].p_tx_buffer = reg_data;
                coines_spi_txrx_desc[bus].tx_length = count;
                coines_spi_txrx_desc[bus].p_rx_buffer = NULL;
                coines_spi_txrx_desc[bus].rx_length = 0;

                error = nrfx_spim_xfer(&coines_spi_instance[bus], &coines_spi_txrx_desc[bus], 0);

                if (error == NRFX_ERROR_INVALID_ADDR)
                {
                    /* The transfer didn't occur because the
                    * SPI peripheral requires that the buffer
                    * is in the RAM Data section
                    */
                    uint8_t temp_buff[count];
                    memcpy(temp_buff, reg_data, count);
                    coines_spi_txrx_desc[bus].p_tx_buffer = temp_buff;

                    error = nrfx_spim_xfer(&coines_spi_instance[bus], &coines_spi_txrx_desc[bus], 0);
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
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_read_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    nrfx_err_t error;

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (coines_is_spi_enabled(bus))
        {
            uint32_t pin_num = multi_io_map[dev_addr];
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

            coines_spi_txrx_desc[bus].p_tx_buffer = &reg_addr;
            coines_spi_txrx_desc[bus].tx_length = 1;
            coines_spi_txrx_desc[bus].p_rx_buffer = NULL;
            coines_spi_txrx_desc[bus].rx_length = 0;

            error = nrfx_spim_xfer(&coines_spi_instance[bus], &coines_spi_txrx_desc[bus], 0);

            if (error == NRFX_SUCCESS)
            {
                coines_spi_txrx_desc[bus].p_tx_buffer = NULL;
                coines_spi_txrx_desc[bus].tx_length = 0;
                coines_spi_txrx_desc[bus].p_rx_buffer = reg_data;
                coines_spi_txrx_desc[bus].rx_length = count;

                error = nrfx_spim_xfer(&coines_spi_instance[bus], &coines_spi_txrx_desc[bus], 0);
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
    uint16_t bytes_read = 0;

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
