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
 * @file    platform.h
 * @brief   This file contains platform layer function prototypes and variable declarations and Macro definitions
 *
 */
/* Note: Document of this file are generated from GenAI GitHub Copilot​ */
#ifndef PLATFORM_H_
#define PLATFORM_H_

/**********************************************************************************/
/* includes */
/**********************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/*********************************************************************/
/* macro definitions */
/**********************************************************************/
#define  SERIAL_READ_BUFF_SIZE                       16384

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/**
 * @brief This API is used to open the serial communication interface with the specified configuration.
 *
 * @param[in] serial_config Pointer to the configuration for the serial communication interface.
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_open_serial(void *serial_config);

/**
 * @brief This API is used to close the serial communication.
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_close_serial(void);

/**
 * @brief This API is used to send data over the serial communication platform.
 *
 * @param[in] buffer Pointer to the data buffer to be sent.
 * @param[in] n_bytes The number of bytes to be sent.
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_send_serial(void *buffer, uint32_t n_bytes);

/**
 * @brief This API is used to send data over the BLE communication platform.
 *
 * @param[in] buffer Pointer to the data buffer to be sent.
 * @param[in] n_bytes The number of bytes to be sent.
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_send_ble(void *buffer, uint32_t n_bytes);

/**
 * @brief This API is used to receive data over the serial communication platform.
 *
 * @param[in] buffer Pointer to the data buffer where the received data will be stored.
 * @param[in] n_bytes The number of bytes to be received.
 * @param[out] n_bytes_read Pointer to a variable where the number of bytes actually read will be stored.
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_receive_serial(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read);

/*!
 * @brief This API is used to scan BLE devices
 *
 * @param[out] ble_info            : Array of structure containing found BLE peripherals' info like Address and Identifier
 * @param[out] peripheral_count    : The number of found BLE peripherals
 * @param[in]  scan_timeout_ms     : Timeout for BLE scan
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_scan_ble(void *ble_info, uint8_t *peripheral_count, size_t scan_timeout_ms);

/**
 * @brief This API is used to open the ble communication interface with the specified configuration.
 *
 * @param[in] ble_config Pointer to the configuration for the ble communication interface.
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_open_ble(void *ble_config);

/**
 * @brief This API is used to close the ble communication platform.
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_close_ble(void);

/**
 * @brief This API is used to receive data over the BLE communication platform.
 *
 * @param[in] buffer Pointer to the data buffer where the received data will be stored.
 * @param[in] n_bytes The number of bytes to be received.
 * @param[out] n_bytes_read Pointer to a variable where the number of bytes actually read will be stored.
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_receive_ble(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read);

/*!
 * @brief This API is used to flush the buffer
 *
 */
void platform_flush_serial(void);

/**
 * @brief This API is used to start the serial read thread.
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_serial_read_thread_start(void);

/**
 * @brief This API is used to stop the serial read thread.
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_serial_read_thread_stop(void);

/**
 * @brief Sets the callback function for BLE notifications.
 *
 * This function sets the callback function to be called when a BLE notification is received.
 *
 * @param attach_cb The callback function to be set.
 */
void platform_ble_attach_notify_cb(bool attach_cb);

/*!
 * @brief Checks the status of the serial port.
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_check_serial_port_status(void);

/*!
 * @brief Checks the status of the ble.
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t platform_check_ble_status(void);

#endif /* PLATFORM_H_ */
