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
 * @file    w25_common.c
 * @date    Apr 30, 2021
 * @brief   W25 flash chip common APIs
 */

/*!
 * @addtogroup
 * @brief
 * @{*/

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "nrfx_spim.h"
#include "nrf_gpio.h"

/**********************************************************************************/
/* own header files */
/**********************************************************************************/
#include "w25_common.h"
#include "w25n01gw.h"
#include "w25n02jw.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/*! Write protect pin of w25n01gw chip */
#define W25_WRITE_PROTECT_PIN   NRF_GPIO_PIN_MAP(0, 22)
/*! Hold pin of w25n01gw chip */
#define W25_HOLD_PIN           NRF_GPIO_PIN_MAP(0, 23)
/*! Chip Select pin of w25n01gw chip */
#define W25_CHIP_SEL_PIN       NRF_GPIO_PIN_MAP(0, 17)

#define SPI_TX_SIZE  (256 + 1)
#define SPI_MOSI_PIN_FLASH                 NRF_GPIO_PIN_MAP(0, 20)
#define SPI_MISO_PIN_FLASH                 NRF_GPIO_PIN_MAP(0, 21)
#define SPI_CLK_PIN_FLASH                  NRF_GPIO_PIN_MAP(0, 19)
#define SPI_CS_PIN_FLASH                   NRF_GPIO_PIN_MAP(0, 17)

/*! Opcode to read the Status Register */
#define W25_CMD_RD_REG                0x0F
/*! Command length for reading from the registers */
#define W25_RD_SREG_CMD_LEN           0x01
/*! First two bytes of send buffer will receive nothing.Hence reading from 3rd byte */
#define W25_MIN_RCV_BYTES_LEN         0x03
/*! Opcode to get JEDEC id */
#define W25_CMD_JDEC_ID               0x9F
/*! Opcode to Reset */
#define W25_CMD_RESET                0xFF

/*! Position of BUSY bit of status register-3 */
#define W25_BUSY_STAT                 (1 << 0)

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/*! Holds the initialization status of the module */
w25_nand_error_t w25_init_status = W25_NAND_UNINITIALIZED;

/*! Holds the spi handle */
uint8_t w25_spi_intf_handle = 0;

const nrfx_spim_t flash_spi_instance = NRFX_SPIM_INSTANCE(2);
nrfx_spim_config_t flash_spi_config = NRFX_SPIM_DEFAULT_CONFIG;
static nrfx_spim_xfer_desc_t flash_spi_xfer;
static uint8_t flash_spi_tx_buff[SPI_TX_SIZE];

/**********************************************************************************/
/* function prototypes */
/**********************************************************************************/
void w25_gpio_pin_output_set(uint8_t pin);
uint8_t w25_spi_init(void);
static void w25_device_reset(void);
static uint8_t w25_read_reg(w25_reg_t reg);
uint8_t w25_spi_rx_tx(uint8_t spi_entity, uint8_t address, uint8_t* tx_buffer,
                      uint16_t tx_count,
                      uint8_t* rx_buffer, uint16_t rx_count);

/*!
 * @brief This function initializes the w25 driver
 *
 */
w25_nand_error_t w25_init(uint16_t *device_id)
{
    w25_nand_error_t ret_code = W25_NAND_INITIALIZATION_FAILED;
    w25_deviceinfo_t info;

    /* If already initialized, return initiliazation success */
    if (w25_spi_intf_handle)
    {
        return W25_NAND_INITIALIZED;
    }

    /*Configure and set the CS pin,WP pin and Hold pin as output*/
    w25_gpio_pin_output_set(W25_WRITE_PROTECT_PIN);
    w25_gpio_pin_output_set(W25_HOLD_PIN);
    w25_gpio_pin_output_set(W25_CHIP_SEL_PIN);

    w25_spi_intf_handle = w25_spi_init();

    if (!w25_spi_intf_handle)
    {
        ret_code = W25_NAND_ERROR;
        return ret_code;
    }

    /*Reset the chip*/
    w25_device_reset();
    /*Wait until all the device is powered up */
    while ((w25_read_reg(W25_STATUS_REG_ADDR)) & W25_BUSY_STAT)
    {
        ;
    }

    w25_get_manufacture_and_devid(&info);

    if (((info.device_id == W25N01GW_DEVICE_ID) || (info.device_id == W25M02GW_DEVICE_ID)) &&
        (info.mfg_id == W25_MANUFACTURER_ID))
    {
        w25n01gw_init_protect_reg();
        w25n01gw_init_config_reg();

        *device_id = info.device_id;
        w25_init_status = W25_NAND_INITIALIZED;
        ret_code = W25_NAND_INITIALIZED;

    }
    else if (((info.device_id == W25N02JW_DEVICE_ID) || (info.device_id == W25N02KW_DEVICE_ID)) && (info.mfg_id == W25_MANUFACTURER_ID))
    {
        w25n02jw_init_protect_reg();
        w25n02jw_init_config_reg();

        *device_id = info.device_id;
        w25_init_status = W25_NAND_INITIALIZED;
        ret_code = W25_NAND_INITIALIZED;
    }
    else
    {
        *device_id = 0;
        ret_code = W25_NAND_INITIALIZATION_FAILED;
    }

    return ret_code;
}

/*!
 * @brief       This function gives the manufacture id and device id
 */
void w25_get_manufacture_and_devid(w25_deviceinfo_t* info)
{
    uint8_t dummy_byte = 0;
    uint8_t recv_buff[5];

    memset(recv_buff, 0, 5);

    /*Gets the manufacture id and device id
     * mfg id and device id together is only 3 bytes.
     * However 5 bytes are read because first two bytes received are not valid data.This is a deviation from the datasheet specification
     * */
    (void)w25_spi_rx_tx(w25_spi_intf_handle, W25_CMD_JDEC_ID, &dummy_byte, W25_RD_SREG_CMD_LEN, &recv_buff[0], 5);

    /*Update the structure*/
    info->mfg_id = recv_buff[2];
    info->device_id = recv_buff[3] << 8 | recv_buff[4];

}
/*!
 * @brief       This function resets the chip
 *
 */
static void w25_device_reset(void)
{
    (void)w25_spi_rx_tx(w25_spi_intf_handle, W25_CMD_RESET, NULL, 0, NULL, 0);
}

/*!
 * @brief       This function sets the gpio pin given as input
 *
 * @retval      none
 */

void w25_gpio_pin_output_set(uint8_t pin)
{
    nrf_gpio_cfg_output(pin);
    nrf_gpio_pin_set(pin);
}

/*!
 * @brief       This function initialises the spi module
 *
 * @retval      spi handle
 */

/*!
 * @brief       This function initialises the spi module
 *
 * @retval      spi handle
 */

uint8_t w25_spi_init()
{
    flash_spi_config.miso_pin = SPI_MISO_PIN_FLASH;
    flash_spi_config.mosi_pin = SPI_MOSI_PIN_FLASH;
    flash_spi_config.sck_pin = SPI_CLK_PIN_FLASH;
    flash_spi_config.ss_pin = SPI_CS_PIN_FLASH;
    flash_spi_config.frequency = NRF_SPIM_FREQ_8M;

    if (nrfx_spim_init(&flash_spi_instance, &flash_spi_config, NULL, NULL) != NRFX_SUCCESS)
    {
        return 0;
    }
    return 1;
}

static uint8_t w25_read_reg(w25_reg_t reg)
{
    uint8_t recv_buff[W25_MIN_RCV_BYTES_LEN] = { 0 };

    (void)w25_spi_rx_tx(w25_spi_intf_handle,
    W25_CMD_RD_REG,
                  (uint8_t *)&reg,
                  W25_RD_SREG_CMD_LEN,
                  &recv_buff[0],
                  W25_MIN_RCV_BYTES_LEN);

    return recv_buff[W25_MIN_RCV_BYTES_LEN - 1];

}

/*!
 * @brief       This function sends and receives the data via SPI
 *
 * @param[in]   spi_entity : SPI handle
 * @param[in]   address : reg address
 * @param[in]   tx_buffer : transmission buffer
 * @param[in]   tx_count : transmission buffer size
 * @param[in]   rx_buffer : receiver buffer
 * @param[in]   rx_count : receiver buffer size
 *
 */

uint8_t w25_spi_rx_tx(uint8_t spi_entity, uint8_t address, uint8_t* tx_buffer, uint16_t tx_count, uint8_t* rx_buffer, uint16_t rx_count)
{
    (void)spi_entity;
    flash_spi_tx_buff[0] = address;
    memcpy(&flash_spi_tx_buff[1], tx_buffer, tx_count);

    flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
    flash_spi_xfer.tx_length = tx_count + 1;
    flash_spi_xfer.p_rx_buffer = rx_buffer;
    flash_spi_xfer.rx_length = rx_count;

    if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
    {
        return 1;
    }

    return 0;
}
