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
 * @file    mcu_app3x_interface_nb.h
 * @date    Nov 17, 2025
 * @brief   This file contains function prototypes for non-blocking interface API's
 */

#ifndef MCU_APP3X_INTERFACE_NB_H_
#define MCU_APP3X_INTERFACE_NB_H_

#include <stdint.h>
#include <stdio.h>

#include "coines.h"
/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/** @brief Argument package for a non-blocking I2C read transaction. */
typedef struct interface_i2c_read_args
{
    enum coines_i2c_bus bus;        /**< Target I2C bus instance. */
    uint8_t device_address;         /**< 7-bit I2C device address. */
    uint8_t *write_buffer;          /**< Preamble/register address bytes (NULL if none). */
    uint8_t *read_bufer;            /**< Destination buffer for read data (must persist until completion). */
    uint16_t write_length;          /**< Number of bytes in write_buffer (0 for no preamble). */
    uint16_t read_length;           /**< Number of bytes to read from device. */
} interface_i2c_read_args_t;

/** @brief Argument package for a non-blocking I2C write transaction. */
typedef struct interface_i2c_write_args
{
    enum coines_i2c_bus bus;        /**< Target I2C bus instance. */
    uint8_t device_address;         /**< 7-bit I2C device address. */
    uint8_t *write_buffer;          /**< Data to be written (must persist until transfer completes). */
    uint16_t write_length;          /**< Length of data in write_buffer. */
} interface_i2c_write_args_t;

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/**
 * @brief Configure the SPI bus in non-blocking mode.
 *
 * @param bus SPI bus identifier.
 * @param spi_speed SPI bus speed.
 * @param spi_mode SPI mode (clock polarity and phase).
 * @return int16_t COINES_SUCCESS on success, error code otherwise.
 */
int16_t coines_config_spi_bus_nb(enum coines_spi_bus bus,
                                        enum coines_spi_speed spi_speed,
                                        enum coines_spi_mode spi_mode);

/**
 * @brief Read data from an SPI device in non-blocking mode.
 *
 * @param bus SPI bus identifier.
 * @param cs_pin Chip select pin.
 * @param reg_addr Register address to read from.
 * @param reg_data Pointer to buffer to store read data.
 * @param count Number of bytes to read.
 * @return int8_t COINES_SUCCESS on success, error code otherwise.
 */
int8_t coines_read_spi_nb(enum coines_spi_bus bus,
                                 uint8_t cs_pin,
                                 uint8_t reg_addr,
                                 uint8_t *reg_data,
                                 uint16_t count);

/**
 * @brief Write data to an SPI device in non-blocking mode.
 *
 * @param bus SPI bus identifier.
 * @param cs_pin Chip select pin.
 * @param reg_addr Register address to write to.
 * @param reg_data Pointer to buffer containing data to write.
 * @param count Number of bytes to write.
 * @return int8_t COINES_SUCCESS on success, error code otherwise.
 */
int8_t coines_write_spi_nb(enum coines_spi_bus bus,
                                  uint8_t cs_pin,
                                  uint8_t reg_addr,
                                  uint8_t *reg_data,
                                  uint16_t count);

/**
 * @brief Write 16-bit data to an SPI device in non-blocking mode.
 *
 * @param bus SPI bus identifier.
 * @param cs_pin Chip select pin.
 * @param reg_addr 16-bit register address to write to.
 * @param reg_data Pointer to buffer containing data to write.
 * @param count Number of 16-bit words to write.
 * @param spi_transfer_bits SPI transfer bit width.
 * @return int8_t COINES_SUCCESS on success, error code otherwise.
 */
int8_t coines_write_16bit_spi_nb(enum coines_spi_bus bus,
                                        uint8_t cs_pin,
                                        uint16_t reg_addr,
                                        void *reg_data,
                                        uint16_t count,
                                        enum coines_spi_transfer_bits spi_transfer_bits);

/**
 * @brief Read 16-bit data from an SPI device in non-blocking mode.
 *
 * @param bus SPI bus identifier.
 * @param cs_pin Chip select pin.
 * @param reg_addr 16-bit register address to read from.
 * @param reg_data Pointer to buffer to store read data.
 * @param count Number of 16-bit words to read.
 * @param spi_transfer_bits SPI transfer bit width.
 * @return int8_t COINES_SUCCESS on success, error code otherwise.
 */
int8_t coines_read_16bit_spi_nb(enum coines_spi_bus bus,
                                       uint8_t cs_pin,
                                       uint16_t reg_addr,
                                       void *reg_data,
                                       uint16_t count,
                                       enum coines_spi_transfer_bits spi_transfer_bits);

/**
 * @brief Deconfigure the SPI bus in non-blocking mode.
 *
 * @param bus SPI bus identifier.
 * @return int16_t COINES_SUCCESS on success, error code otherwise.
 */
int16_t coines_deconfig_spi_bus_nb(enum coines_spi_bus bus);

/**
 * @brief Issue a non-blocking I2C write-only transaction.
 *
 * @param[in] i2c_write_params Pointer to populated write argument structure.
 *                             Must not be NULL; write_buffer must be valid for
 *                             the duration of the transfer.
 *
 * @return int8_t COINES_SUCCESS on success, error code otherwise.
 */
int8_t coines_i2c_write_nonblock(interface_i2c_write_args_t *i2c_write_params);

/**
 * @brief Issue a non-blocking I2C combined write+read transaction.
 *
 * @param[in] i2c_read_params Pointer to populated read argument structure.
 *                            Must not be NULL. Buffers referenced inside must
 *                            remain valid until the completion callback fires.
 *
 * @return int8_t COINES_SUCCESS on success, error code otherwise.
 */
int8_t coines_i2c_read_nonblock(interface_i2c_read_args_t *i2c_read_params);

/**
 * @brief Register a callback invoked when a non-blocking I2C transaction completes.
 *
 * @param[in] bus      I2C bus instance to attach the callback to.
 * @param[in] callback Function pointer to invoke on completion.
 *
 * @return int8_t COINES_SUCCESS on success, error code otherwise.
 */
int8_t coines_i2c_bus_register_transaction_completed_callbacks(enum coines_i2c_bus bus, void (*callback)(bool));

#endif /* MCU_APP3X_INTERFACE_NB_H_ */