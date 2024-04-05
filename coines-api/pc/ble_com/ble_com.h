/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  @file   ble_com.h
 *  @brief  This file contains BLE communication interface related function prototype, variable declarations and Macro definitions
 */

#ifndef BLE_COM_H
#define BLE_COM_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <coines.h>

/**********************************************************************************/
/* typedef definitions */
/**********************************************************************************/

typedef enum ble_error_code_types {
    BLE_COM_OK                     = 0,
    BLE_COM_E_ADAPTOR_NOT_FOUND    = -1,
    BLE_COM_E_CONNECT_FAILED       = -2,
    BLE_COM_E_WRITE_FAILED         = -3,
    BLE_COM_E_BLE_LIB_NOT_LOADED         = -4,
    BLE_COM_E_PERIPHERAL_NOT_FOUND  = -5,
    BLE_COM_E_INVALID_COM_CONFIG = -6,
    BLE_COM_E_TX_NOTIFY_FAILED = -7,
    BLE_COM_E_ADAPTER_BLUETOOTH_NOT_ENABLED = -8,
    BLE_COM_E_DISCONNECT_FAILED = -9,
    BLE_COM_E_PERIPHERAL_NOT_CONNECTED = -10,
    BLE_COM_E_APP_BOARD_NOT_FOUND = -11,
} ble_error_code;

typedef enum ble_index_keys {
    IDENTIFIER,
    ADDRESS,
    CLOSEST_APP_BOARD
} ble_index_key;

/**********************************************************************************/
/* function declarations */
/**********************************************************************************/

/*!
 * @brief Connects to BLE Adapter and returns list of BLE peripherals by initializing dll load
 *
 * @param[out] ble_info            : Array of structure containing found BLE peripherals' info like Address and Identifier
 * @param[out] peripheral_count    : The number of found BLE peripherals
 * @param[in]  scan_timeout_ms     : Timeout for BLE scan
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Failure.
 */
int8_t ble_scan(struct ble_peripheral_info *ble_info, uint8_t *peripheral_count, size_t scan_timeout_ms);

/*!
 * @brief Establishes connection to BLE peripheral with peripheral index
 *
 * @param[in] ble_peripheral_name : Name of the BLE peripheral to be connected
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Failure.
 */
int8_t ble_connect(struct ble_peripheral_info *ble_config);

/*!
 * @brief Performs write operation using BLE interface
 *
 * @param[in] buffer  : data
 * @param[in] n_bytes : number of bytes to send
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Failure.
 */
int8_t ble_write(void *buffer, uint32_t n_bytes);

/*!
 * @brief Sets buffer with data received on BLE TX Notify
 *
 * @param[out] buffer       : buffer to store the received data
 * @param[in] n_bytes       : number of bytes to read
 * @param[out] n_bytes_read : number of bytes read
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Failure.
 */
int8_t ble_read(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read);

/*!
 * @brief Performs close operation of the BLE communication.
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Failure.
 *
 */
int8_t ble_close(void);

#ifdef __cplusplus
}
#endif

#endif
