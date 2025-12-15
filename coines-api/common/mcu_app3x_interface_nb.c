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
 * @file    mcu_app3x_interface_nb.c
 * @date    Jun 10, 2025
 * @brief   SPI non-blocking interface for mcu_app3x
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"
#include "mcu_app3x_interface.h"
#include "mcu_app3x_support.h"
#include "mcu_app3x_interface_nb.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define SPI_DMA_TX_BUFFER_SIZE 1024

/**********************************************************************************/
#define SPI_DMA_TRANSFER_SUCCESS
#define SPI_DMA_TRANSFER_ERROR

#define SPI_DMA_TRANSFER_IDLE
#define SPI_DMA_TRANSFER_PENDING

/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
uint8_t spi_dma_tx_data_buffer[SPI_DMA_TX_BUFFER_SIZE];
uint8_t spi_dma_cs_pin_num;
volatile bool spi_dma_status_busy_flag = false;
volatile bool spi_nb_xfer_done = false;
extern coines_spi_intf_t coines_spi_intf[COINES_SPI_BUS_MAX];
extern coines_i2c_intf_t coines_i2c_intf[COINES_I2C_BUS_MAX];
static void (*i2c_transaction_callbacks[COINES_I2C_BUS_MAX])(bool) = {0};
extern int16_t coines_get_spi_instance(enum coines_spi_bus bus);
extern bool coines_is_spi_enabled(enum coines_spi_bus bus);
extern int16_t coines_set_spi_instance(enum coines_spi_bus bus, uint8_t enable);
extern uint8_t prepare_reg_addr_buffer(uint8_t *reg_addr_buffer, uint16_t reg_addr, uint8_t transfer_bits);

/**********************************************************************************/
/* static variables */
/**********************************************************************************/

/**********************************************************************************/
/* static function declartions */
/**********************************************************************************/
static void spi_nb_event_handler(nrfx_spim_evt_t const *p_event, void *p_context);
static int8_t spi_read_write(enum coines_spi_bus bus, uint8_t *tx_buff, uint32_t tx_len, uint8_t *rx_buff, uint32_t rx_len);
static bool is_spi_busy(void);

/**********************************************************************************/
/* Functions */
/**********************************************************************************/

/*lint -e522 */
__attribute__((weak)) void coines_spi_xfer_completed(bool status)
{
}

static inline bool is_spi_busy(void)
{
    bool status = false;
    CRITICAL_REGION_ENTER();
    if (spi_dma_status_busy_flag)
    {
        status = true;
    }
    CRITICAL_REGION_EXIT();
    return status;
}

static inline void update_spi_busy_flag(bool status)
{
    CRITICAL_REGION_ENTER();
    spi_dma_status_busy_flag = status;
    CRITICAL_REGION_EXIT();
}

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_nb_event_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
    (void)p_context;

    if (spi_dma_cs_pin_num)
    {
        /* Deactivate CS pin */
        nrf_gpio_pin_write(spi_dma_cs_pin_num, 1);
        spi_dma_cs_pin_num = 0;
    }
    spi_dma_status_busy_flag = false;

    switch (p_event->type)
    {
    case NRFX_SPIM_EVENT_DONE:
        spi_nb_xfer_done = true;
        coines_spi_xfer_completed(spi_nb_xfer_done); /* Notify the application that SPI transfer is completed */
        break;
    default:
        spi_nb_xfer_done = false;
        coines_spi_xfer_completed(spi_nb_xfer_done);
        break;
    }
}

/*!
 *  @brief This API is used to configure the SPI bus
 */
int16_t coines_config_spi_bus_nb(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
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

#define COINES_NRF_SPEED_MAP(coines_spi, nrf_spi)                        \
    case COINES_SPI_SPEED_##coines_spi:                                  \
        coines_spi_intf[bus].config.frequency = NRF_SPIM_FREQ_##nrf_spi; \
        break

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
                coines_spi_intf[bus].config.frequency = NRF_SPIM_FREQ_2M;
            }
            if (NRFX_SUCCESS == nrfx_spim_init(&coines_spi_intf[bus].peripheral_instance, &coines_spi_intf[bus].config, spi_nb_event_handler, NULL))
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
                coines_spi_intf[bus].enabled = false; /* Set SPI bus status to disabled */
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
int16_t coines_deconfig_spi_bus_nb(enum coines_spi_bus bus)
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
 *  @brief This API is used to write the data in SPI communication.
 */

int8_t spi_write_nb(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, uint8_t *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    nrfx_err_t error;
    uint8_t reg_addr_length = 0;
    uint32_t pin_no;
    int8_t result = COINES_SUCCESS;

    if (is_spi_busy())
    {
        return COINES_E_SPI_BUS_BUSY;
    }

    if ((bus < COINES_SPI_BUS_MAX) && (bus >= COINES_SPI_BUS_0))
    {
        if (coines_is_spi_enabled(bus))
        {
            pin_no = multi_io_map[cs_pin];
            if (pin_no == 0 || pin_no == 0xff)
            {
                return COINES_E_FAILURE;
            }

            if (reg_data == NULL || count == 0)
            {
                return COINES_E_INVALID_INPUT;
            }

            reg_addr_length = prepare_reg_addr_buffer(spi_dma_tx_data_buffer, reg_addr, spi_transfer_bits);

            if ((reg_addr_length + count) >= SPI_DMA_TX_BUFFER_SIZE)
            {
                return COINES_E_INSUFFICIENT_BUFFER;
            }

            memcpy(&spi_dma_tx_data_buffer[reg_addr_length], reg_data, count);
            update_spi_busy_flag(true);
            nrf_gpio_cfg_output(pin_no);
            /* Activate CS pin */
            nrf_gpio_pin_write(pin_no, 0);

            spi_dma_cs_pin_num = pin_no;

            error = spi_read_write(bus, spi_dma_tx_data_buffer, reg_addr_length + count, NULL, 0);

            if (error == NRFX_SUCCESS)
            {
                result = COINES_SUCCESS;
            }
            else
            {
                update_spi_busy_flag(false);
                nrf_gpio_pin_write(pin_no, 0);
                result = COINES_E_FAILURE;
            }
        }
        else
        {
            result = COINES_E_SPI_BUS_NOT_ENABLED;
        }
    }
    else
    {
        result = COINES_E_SPI_INVALID_BUS_INTF;
    }

    return result;
}

/**
 * @brief Reads data from an SPI device in non-blocking mode.
 *
 * @param[in]  bus               SPI bus identifier
 * @param[in]  cs_pin            Chip select pin
 * @param[in]  reg_addr          Register address
 * @param[out] reg_data          Pointer to data buffer to store read data
 * @param[in]  count             Number of bytes to read
 * @param[in]  spi_transfer_bits SPI transfer bit mode
 *
 * @return COINES_SUCCESS on success, error code otherwise
 */
static int8_t spi_read_nb(enum coines_spi_bus bus,
                          uint8_t cs_pin,
                          uint16_t reg_addr,
                          uint8_t *reg_data,
                          uint16_t count,
                          enum coines_spi_transfer_bits spi_transfer_bits)
{
    nrfx_err_t error;
    uint8_t reg_addr_length;
    uint32_t pin_num;

    if (is_spi_busy())
    {
        return COINES_E_SPI_BUS_BUSY;
    }

    if ((bus >= COINES_SPI_BUS_MAX) || (bus < COINES_SPI_BUS_0))
    {
        return COINES_E_SPI_INVALID_BUS_INTF;
    }
    if (!coines_is_spi_enabled(bus))
    {
        return COINES_E_SPI_BUS_NOT_ENABLED;
    }

    pin_num = multi_io_map[cs_pin];
    if ((pin_num == 0U) || (pin_num == 0xFFU))
    {
        return COINES_E_FAILURE;
    }

    if ((reg_data == NULL) || (count == 0U))
    {
        return COINES_E_INVALID_INPUT;
    }
    nrf_gpio_cfg_output(pin_num);

    update_spi_busy_flag(true);
    nrf_gpio_pin_write(pin_num, 0);
    spi_dma_cs_pin_num = pin_num;

    reg_addr_length = prepare_reg_addr_buffer(spi_dma_tx_data_buffer, reg_addr, spi_transfer_bits);

    error = spi_read_write(bus, spi_dma_tx_data_buffer, reg_addr_length, reg_data, count);

    if (error != NRFX_SUCCESS)
    {
        update_spi_busy_flag(false);
        nrf_gpio_pin_write(pin_num, 0);
        return COINES_E_FAILURE;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 */
int8_t coines_write_spi_nb(enum coines_spi_bus bus, uint8_t cs_pin, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    return spi_write_nb(bus, cs_pin, (uint16_t)reg_addr, reg_data, count, COINES_SPI_TRANSFER_8BIT);
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 */
int8_t coines_read_spi_nb(enum coines_spi_bus bus, uint8_t cs_pin, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    return spi_read_nb(bus, cs_pin, (uint16_t)reg_addr, reg_data, count, COINES_SPI_TRANSFER_8BIT);
}

/*!
 *  @brief This API is used to write 16-bit register data on the SPI device.
 */
int8_t coines_write_16bit_spi_nb(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    uint16_t swapped_reg_data[count];

    if (spi_transfer_bits == COINES_SPI_TRANSFER_16BIT)
    {
        /* Conversion of payload from big endian to little endian */
        swap_endianness(swapped_reg_data, (uint16_t *)reg_data, count);

        return spi_write_nb(bus, cs_pin, reg_addr, (uint8_t *)swapped_reg_data, (uint16_t)(count * sizeof(uint16_t)), COINES_SPI_TRANSFER_16BIT);
    }
    else
    {
        return spi_write_nb(bus, cs_pin, reg_addr, (uint8_t *)reg_data, count, COINES_SPI_TRANSFER_16BIT);
    }
}

/*!
 *  @brief This API is used to read 16-bit register data from the SPI device.
 */
int8_t coines_read_16bit_spi_nb(enum coines_spi_bus bus, uint8_t cs_pin, uint16_t reg_addr, void *reg_data, uint16_t count, enum coines_spi_transfer_bits spi_transfer_bits)
{
    int8_t ret;

    if (spi_transfer_bits == COINES_SPI_TRANSFER_16BIT)
    {
        ret = spi_read_nb(bus, cs_pin, reg_addr, (uint8_t *)reg_data, (uint16_t)(count * sizeof(uint16_t)), COINES_SPI_TRANSFER_16BIT);
        /* Conversion of payload from big endian to little endian */
        swap_endianness((uint16_t *)reg_data, (uint16_t *)reg_data, count);
    }
    else
    {
        ret = spi_read_nb(bus, cs_pin, reg_addr, (uint8_t *)reg_data, count, COINES_SPI_TRANSFER_16BIT);
    }

    return ret;
}

/**
 * @brief This API is used to perform a SPI read/write transaction (non-blocking).
 */
int8_t spi_read_write(enum coines_spi_bus bus, uint8_t *tx_buff, uint32_t tx_len, uint8_t *rx_buff, uint32_t rx_len)
{
    int8_t rslt;

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_buff, tx_len, rx_buff, rx_len);

    spi_nb_xfer_done = false;
    /* Perform SPI transaction */
    rslt = nrfx_spim_xfer(&coines_spi_intf[bus].peripheral_instance, &xfer_desc, NRFX_SPIM_FLAG_TX_POSTINC | NRFX_SPIM_FLAG_RX_POSTINC);

    return (rslt == NRFX_SUCCESS) ? COINES_SUCCESS : COINES_E_FAILURE;
}

/***************************************************************************************************************************** */

/** @brief Issue a non-blocking I2C write-only transaction. */
int8_t coines_i2c_write_nonblock(interface_i2c_write_args_t *i2c_write_params)
{
    nrfx_err_t error;
    if ((i2c_write_params->bus < COINES_I2C_BUS_MAX) && (i2c_write_params->bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(i2c_write_params->bus))
        {
            /*lint -e785*/
            nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(i2c_write_params->device_address, i2c_write_params->write_buffer, i2c_write_params->write_length);
            /*lint +e785*/
            error = nrfx_twim_xfer(&coines_i2c_intf[i2c_write_params->bus].peripheral_instance, &write_desc, NRFX_TWIM_FLAG_TX_POSTINC);
            if (error == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
    }
    return COINES_E_FAILURE;
}

/** @brief Issue a non-blocking I2C combined write+read transaction. */
int8_t coines_i2c_read_nonblock(interface_i2c_read_args_t *i2c_read_params)
{
    nrfx_err_t error_code;

    if ((i2c_read_params->bus < COINES_I2C_BUS_MAX) && (i2c_read_params->bus >= COINES_I2C_BUS_0))
    {
        if (coines_is_i2c_enabled(i2c_read_params->bus))
        {
            nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_TXRX(i2c_read_params->device_address,
                                                                       i2c_read_params->write_buffer, 
                                                                       i2c_read_params->write_length, 
                                                                       i2c_read_params->read_bufer, 
                                                                       i2c_read_params->read_length);

            error_code = nrfx_twim_xfer(&coines_i2c_intf[i2c_read_params->bus].peripheral_instance,
                                        &read_desc,
                                        NRFX_TWIM_FLAG_RX_POSTINC | NRFX_TWIM_FLAG_REPEATED_XFER);

            if (error_code == NRFX_SUCCESS)
            {
                return COINES_SUCCESS;
            }
            else
            {
                return COINES_E_FAILURE;
            }
        }
        return COINES_E_FAILURE;
    }
    return COINES_E_FAILURE;
}

/** @brief Register a callback invoked when a non-blocking I2C transaction completes. */
int8_t coines_i2c_bus_register_transaction_completed_callbacks(enum coines_i2c_bus bus, void (*callback)(bool))
{
    if ((bus < COINES_I2C_BUS_MAX)&&(callback!=NULL))
    {
        i2c_transaction_callbacks[bus] = callback;
        return COINES_SUCCESS;
    }
    return COINES_E_FAILURE;
}

/**
 * @brief Callback function invoked upon completion of an I2C bus transaction.
 *
 * This function is called when an I2C transaction on the specified bus has finished.
 *
 * @param bus    The I2C bus on which the transaction was performed. 
 *               This is specified by the coines_i2c_bus enumeration.
 * @param status Boolean value indicating the result of the transaction.
 *               - true: Transaction completed successfully.
 *               - false: Transaction failed.
 */
void coines_i2c_bus_transaction_complete_callback(enum coines_i2c_bus bus, bool status)
{
    if ((bus < COINES_I2C_BUS_MAX)&&(i2c_transaction_callbacks[bus]!=NULL))
    {
        i2c_transaction_callbacks[bus](status);
    }
}
