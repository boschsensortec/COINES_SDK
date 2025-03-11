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
 *  @file   coines_common.c
 *  @brief  This file contains common functions and declarations for LEGACY and COINES Bridge
 */

/*********************************************************************/
/* system header files */
/**********************************************************************/
#include <stdint.h>
#include <string.h>

/*********************************************************************/
/* own header files */
/**********************************************************************/
#include "coines_common.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define SWAP_BYTES(x) ((((x) >> 8) | ((x) << 8)) & 0xffff)

/*********************************************************************/
/* Function definitions */
/**********************************************************************/

/*!
 * @brief This API is used to COINES_SDK error codes to error strings
 */
const char *get_coines_error_str(int16_t error_code)
{
    static const error_code_mapping error_mappings[] = {
        { COINES_SUCCESS, "Success" }, { COINES_E_FAILURE, "[COINES Error] Generic failure" },
        { COINES_E_COMM_IO_ERROR, "[COINES Error] Communication IO failed. Check connections with the sensor" },
        { COINES_E_UNABLE_CLAIM_INTF, "[COINES Error] Unable to claim interface. Check if the board is in use" },
        { COINES_E_COMM_INIT_FAILED, "[COINES Error] Communication initialization failed" },
        { COINES_E_UNABLE_OPEN_DEVICE, "[COINES Error] Failure to open device" },
        { COINES_E_DEVICE_NOT_FOUND, "[COINES Error] Device not found" },
        { COINES_E_MEMORY_ALLOCATION, "[COINES Error] Failure to allocate memory" },
        { COINES_E_NOT_SUPPORTED, "[COINES Error] Feature not supported" },
        { COINES_E_NULL_PTR, "[COINES Error] Null pointer" },
        { COINES_E_COMM_WRONG_RESPONSE, "[COINES Error] Wrong response" },
        { COINES_E_SPI16BIT_NOT_CONFIGURED, "[COINES Error] SPI not configured for 16-bit mode" },
        { COINES_E_SPI_INVALID_BUS_INTF, "[COINES Error] Invalid SPI bus interface" },
        { COINES_E_SPI_CONFIG_EXIST, "[COINES Error] SPI instance already configured" },
        { COINES_E_SPI_BUS_NOT_ENABLED, "[COINES Error] SPI bus not enabled" },
        { COINES_E_SPI_CONFIG_FAILED, "[COINES Error] SPI instance configuration failed" },
        { COINES_E_I2C_INVALID_BUS_INTF, "[COINES Error] Invalid I2C bus interface" },
        { COINES_E_I2C_BUS_NOT_ENABLED, "[COINES Error] I2C bus not enabled" },
        { COINES_E_I2C_CONFIG_FAILED, "[COINES Error] I2C instance configuration failed" },
        { COINES_E_I2C_CONFIG_EXIST, "[COINES Error] I2C instance already configured" },
        { COINES_E_TIMER_INIT_FAILED, "[COINES Error] Timer initialization failed" },
        { COINES_E_TIMER_INVALID_INSTANCE, "[COINES Error] Invalid timer instance" },
        { COINES_E_TIMER_CC_CHANNEL_NOT_AVAILABLE, "[COINES Error] Timer CC channel not available" },
        { COINES_E_EEPROM_RESET_FAILED, "[COINES Error] EEPROM reset failed" },
        { COINES_E_EEPROM_READ_FAILED, "[COINES Error] EEPROM read failed" },
        { COINES_E_INIT_FAILED, "[COINES Error] Initialization failed" },
        { COINES_E_STREAM_NOT_CONFIGURED, "[COINES Error] Streaming not configured" },
        { COINES_E_STREAM_INVALID_BLOCK_SIZE, "[COINES Error] Streaming invalid block size" },
        { COINES_E_STREAM_SENSOR_ALREADY_CONFIGURED, "[COINES Error] Streaming sensor already configured" },
        { COINES_E_STREAM_CONFIG_MEMORY_FULL, "[COINES Error] Streaming sensor config memory full" },
        { COINES_E_INVALID_PAYLOAD_LEN, "[COINES Error] Invalid payload length" },
        { COINES_E_CHANNEL_ALLOCATION_FAILED, "[COINES Error] Channel allocation failed" },
        { COINES_E_CHANNEL_DEALLOCATION_FAILED, "[COINES Error] Channel deallocation failed" },
        { COINES_E_CHANNEL_ASSIGN_FAILED, "[COINES Error] Channel assignment failed" },
        { COINES_E_CHANNEL_ENABLE_FAILED, "[COINES Error] Channel enable failed" },
        { COINES_E_CHANNEL_DISABLE_FAILED, "[COINES Error] Channel disable failed" },
        { COINES_E_INVALID_PIN_NUMBER, "[COINES Error] Invalid GPIO pin number" },
        { COINES_E_MAX_SENSOR_COUNT_REACHED, "[COINES Error] Maximum sensor count reached" },
        { COINES_E_EEPROM_WRITE_FAILED, "[COINES Error] EEPROM write failed" },
        { COINES_E_INVALID_EEPROM_RW_LENGTH, "[COINES Error] Invalid EEPROM write length" },
        { COINES_E_SCOM_INVALID_CONFIG, "[COINES Error] Invalid serial com config" },
        { COINES_E_BLE_INVALID_CONFIG, "[COINES Error] Invalid BLE config" },
        { COINES_E_SCOM_PORT_IN_USE, "[COINES Error] Serial com port in use" },
        { COINES_E_INCOMPATIBLE_FIRMWARE, "[COINES Error] Incompatible firmware for the selected comm type" },
        { COINES_E_UART_INIT_FAILED, "[COINES Error] UART initialization failed" },
        { COINES_E_UART_WRITE_FAILED, "[COINES Error] UART write failed" },
        { COINES_E_UART_INSTANCE_NOT_SUPPORT, "[COINES Error] UART instance not supported"},
        { COINES_E_BLE_ADAPTOR_NOT_FOUND, "[COINES Error] BLE Adaptor not found" },
        { COINES_E_BLE_ADAPTER_BLUETOOTH_NOT_ENABLED, "[COINES Error] Adaptor Bluetooth not enabled" },
        { COINES_E_BLE_PERIPHERAL_NOT_FOUND, "[COINES Error] BLE peripheral not found" },
        { COINES_E_BLE_LIBRARY_NOT_LOADED, "[COINES Error] BLE library not loaded" },
        { COINES_E_BLE_APP_BOARD_NOT_FOUND, "[COINES Error] APP board BLE not found" },
        { COINES_E_BLE_COMM_FAILED, "[COINES Error] BLE communication failed" },
		{ COINES_E_DECODER_FAILED, "[COINES Error] Decoder failed" },
        { COINES_E_ENCODER_FAILED, "[COINES Error] Encoder failed" },
        { COINES_E_SERIAL_COMM_FAILED, "[COINES Error] Serial communication failed" },
        { COINES_E_INTERFACE_FAILED, "[COINES Error] Interface failed" },
        { COINES_E_VDD_CONFIG_FAILED, "[COINES Error] VDD configuration is not set min" },
        { COINES_E_VDDIO_CONFIG_FAILED, "[COINES Error] VDDIO configuration is not set min" },
		{ COINES_E_PTHREAD_FAILED, "[COINES Error] Comm Pthread failed" },
        { COINES_E_READ_TIMEOUT, "[COINES Error] Read timeout" },
        { COINES_E_STREAMING_INIT_FAILURE, "[COINES Error] Streaming not initialized" },
        { COINES_E_INVALID_PARAM, "[COINES Error] Invalid parameter" }
    };

    for (uint16_t i = 0; i < sizeof(error_mappings)/sizeof(error_mappings[0]); i++)
    {
        if (error_code == error_mappings[i].code)
        {
            return error_mappings[i].message;
        }
    }

    return "[COINES error] Unknown error code";
}

/*!
 * @brief This API is used to swap the endianness of the 16-bit data
 */
void swap_endianness(uint16_t* dest_arr, uint16_t* src_arr, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) {
        dest_arr[i] = SWAP_BYTES(src_arr[i]);
    }
}

/*!
 *  @brief This API is used to check if the system is little-endian or big-endian
 *
 */
bool is_system_little_endian() {
    uint16_t value = 0x1;  
    // 2-byte integer with the least significant byte set to 1
    // Pointer to the first byte of 'value'
    uint8_t *byte_ptr = (uint8_t*)&value;  
    // If the least significant byte is 1, the system is little-endian
    return byte_ptr[0] == 1;
}
