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
 * @file   ble_com.c
 * @brief  This module provides BLE communication interface layer between host and application board
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#ifdef PLATFORM_LINUX

/* To enable usleep in glibc */
#define _DEFAULT_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#ifdef PLATFORM_WINDOWS
#include <windows.h>
#else
#include <unistd.h>
#include <ctype.h>
#endif

/*********************************************************************/
/* own header files */
/**********************************************************************/
#include "ble_com.h"

/*********************************************************************/
/* Other header files */
/**********************************************************************/
#ifdef PLATFORM_WINDOWS
#include "simpleble_lib_loader.h"
#else
#include <simpleble-0.6.0/simpleble_c/simpleble.h>
#include <simpleble-0.6.0/simpleble_c/logging.h>
#endif

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/
#define BLE_PERIPHERAL_LIST_SIZE  UINT8_C(40)
#define MIN_SCAN_TIMEOUT_MS       5000
#define COM_READ_BUFF             2048
#define COINES_WRITE_CMD          165
#define BLE_TX_NOTIFY_TIMEOUT     30

#define NORDIC_UART_SERVICE_UUID  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NORDIC_UART_CHAR_RX       "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NORDIC_UART_CHAR_TX       "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define APP_BOARD_NAME            "APP Board"

/*********************************************************************/
/* global variables */
/*********************************************************************/
simpleble_uuid_t nordic_uart_service_uuid;
simpleble_uuid_t nordic_uart_char_rx;
simpleble_uuid_t nordic_uart_char_tx;
pthread_mutex_t ble_peripheral_mutex = PTHREAD_MUTEX_INITIALIZER;

/**********************************************************************************/
/* struct declarations */
/**********************************************************************************/

struct local_ble_peripheral_info
{
    char ble_address[COINES_CHAR_MAX_LEN]; /*< BLE device address */
    char ble_identifier[COINES_CHAR_MAX_LEN]; /*< BLE device identifier */
    int16_t ble_rssi;
    simpleble_peripheral_t ble_peripheral;
};

/*********************************************************************/
/* static variables */
/*********************************************************************/

static simpleble_peripheral_t selected_peripheral = NULL;
static volatile uint8_t peripheral_list_len = 0;
static simpleble_adapter_t selected_adapter = NULL;
static uint8_t device_response_buffer[COM_READ_BUFF] = { 0 };
static bool has_tx_notified = false;
static int8_t ble_scan_result = BLE_COM_OK;
static volatile bool is_ble_peripheral_connected = false;
static struct local_ble_peripheral_info ble_peripheral_info_list[BLE_PERIPHERAL_LIST_SIZE];
static bool is_write_data_chunked = false;
static uint32_t expected_write_data_length = 0;
static uint32_t actual_write_data_length = 0;
static volatile bool ble_scan_completed = false;
static volatile uint16_t write_idx = 0, read_idx = 0;


/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static void clean_on_exit(void);
static void adapter_on_scan_start(simpleble_adapter_t adapter, void* userdata);
static void adapter_on_scan_stop(simpleble_adapter_t adapter, void* userdata);
static void adapter_on_scan_found(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata);
static void peripheral_on_notify(simpleble_uuid_t service,
                                 simpleble_uuid_t characteristic,
                                 const uint8_t *data,
                                 size_t data_length,
                                 void* userdata);

static void peripheral_on_connect(simpleble_peripheral_t peripheral, void *userdata);
static void peripheral_on_disconnect(simpleble_peripheral_t peripheral, void *userdata);
static int8_t ble_notify(void);
static int8_t connect_to_ble_peripheral(int8_t ble_peripheral_index);
static void wait_for_tx_notify(void);
static uint32_t read_from_circular_buffer(uint8_t *buffer, uint32_t n_bytes);
static void write_to_circular_buffer(uint8_t *data, uint8_t data_length);
static uint32_t get_buffer_free_space(void);

/*********************************************************************/
/* Local functions */
/*********************************************************************/

static int compare_rssi(const void *a, const void *b)
{
    const struct local_ble_peripheral_info *p1 = (const struct local_ble_peripheral_info *)a;
    const struct local_ble_peripheral_info *p2 = (const struct local_ble_peripheral_info *)b;

    return p2->ble_rssi - p1->ble_rssi;
}

/*!
 * @brief Function to convert a string to lowercase
 *
 */
static void convert_to_lower_case(char *str)
{
    for (size_t i = 0; str[i]; i++)
    {
        str[i] = (char)tolower((unsigned char)str[i]);
    }
}

/*!
 * @brief Function to compare two strings case-insensitively
 *
 */
static bool case_insensitive_compare(char *str1, char *str2)
{
    /* Create copies of the strings, because convert_to_lower_case modifies its argument */
    char str1Copy[strlen(str1) + 1];
    char str2Copy[strlen(str2) + 1];

    strcpy(str1Copy, str1);
    strcpy(str2Copy, str2);

    /* Convert both strings to lowercase */
    convert_to_lower_case(str1Copy);
    convert_to_lower_case(str2Copy);

    /* Compare the lowercase strings */
    return strcmp(str1Copy, str2Copy) == 0;

}

/*!
 * @brief Function to get the BLE peripheral index using key
 *
 */
static int8_t get_ble_index_by_key(ble_index_key key, void *value)
{
    char* peripheral_identifier;
    char* peripheral_address;

    for (int8_t i = 0; i < peripheral_list_len; i++)
    {
        peripheral_identifier = ble_peripheral_info_list[i].ble_identifier;
        peripheral_address = ble_peripheral_info_list[i].ble_address;
        switch (key)
        {
            case ADDRESS:
                if (case_insensitive_compare(peripheral_address, (char *)value))
                {
                    return i;
                }

                break;
            case IDENTIFIER:
                if (case_insensitive_compare(peripheral_identifier, (char *)value))
                {
                    return i;
                }

                break;
            case CLOSEST_APP_BOARD:
                if (strncmp(peripheral_identifier, APP_BOARD_NAME, strlen(APP_BOARD_NAME)) == 0)
                {
                    return i;
                }

                break;
            default:

                return -1;
        }
    }

    return -1;
}

/*!
 * @brief callback function on peripheral connect
 *
 */
static void peripheral_on_connect(simpleble_peripheral_t peripheral, void *userdata)
{
    (void)peripheral;
    (void)userdata;
    pthread_mutex_lock(&ble_peripheral_mutex);
    is_ble_peripheral_connected = true;
    pthread_mutex_unlock(&ble_peripheral_mutex);
    printf("\nBLE connection status: Connected\n");
}

/*!
 * @brief Cleans and deinitializes loaded dll on peripheral disconnect
 *
 */
static void peripheral_on_disconnect(simpleble_peripheral_t peripheral, void *userdata)
{
    (void)peripheral;
    (void)userdata;
    pthread_mutex_lock(&ble_peripheral_mutex);
    is_ble_peripheral_connected = false;
    ble_scan_completed = false;
    pthread_mutex_unlock(&ble_peripheral_mutex);
//     clean_on_exit();
// #ifdef PLATFORM_WINDOWS
//     deinit_dll();
// #endif
}

/*!
 * @brief connects to the peripheral whose index is given as ble_peripheral_index
 */
static int8_t connect_to_ble_peripheral(int8_t ble_peripheral_index)
{
    int8_t error_code;
    char *peripheral_identifier, *peripheral_address;

    selected_peripheral = ble_peripheral_info_list[ble_peripheral_index].ble_peripheral;
    peripheral_identifier = ble_peripheral_info_list[ble_peripheral_index].ble_identifier;
    peripheral_address = ble_peripheral_info_list[ble_peripheral_index].ble_address;
    printf("\nBLE connection status: Connecting to %s [%s]\n", peripheral_identifier, peripheral_address);
    (void)simpleble_peripheral_set_callback_on_connected(selected_peripheral, peripheral_on_connect, NULL);
    (void)simpleble_peripheral_set_callback_on_disconnected(selected_peripheral, peripheral_on_disconnect, NULL);

    error_code = simpleble_peripheral_connect(selected_peripheral);

    return error_code;
}

/*!
 * @brief Releases all simpleble library handles on BLE peripheral disconnection
 */
static void clean_on_exit(void)
{
    /* Release all saved peripherals */
    for (int8_t i = 0; i < peripheral_list_len; i++)
    {
        simpleble_peripheral_release_handle(ble_peripheral_info_list[i].ble_peripheral);
    }

    /* Release the associated handle. */
    simpleble_adapter_release_handle(selected_adapter);
}

/*!
 * @brief callback function to get the list of found BLE peripherals on Adapter found
 */
static void adapter_on_scan_found(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata)
{
    (void)userdata;
    char *adapter_identifier, *peripheral_identifier, *peripheral_address;

    adapter_identifier = simpleble_adapter_identifier(adapter);
    peripheral_identifier = simpleble_peripheral_identifier(peripheral);
    peripheral_address = simpleble_peripheral_address(peripheral);
    int16_t peripheral_rssi = simpleble_peripheral_rssi(peripheral);

    if (adapter_identifier == NULL || peripheral_identifier == NULL || peripheral_address == NULL)
    {
        return;
    }

    pthread_mutex_lock(&ble_peripheral_mutex);
    if (peripheral_list_len < BLE_PERIPHERAL_LIST_SIZE)
    {
        /* Save the peripheral */
        strcpy(ble_peripheral_info_list[peripheral_list_len].ble_identifier, peripheral_identifier);
        strcpy(ble_peripheral_info_list[peripheral_list_len].ble_address, peripheral_address);
        ble_peripheral_info_list[peripheral_list_len].ble_rssi = peripheral_rssi;
        ble_peripheral_info_list[peripheral_list_len].ble_peripheral = peripheral;
        peripheral_list_len++;
    }
    else
    {
        /* Release the associated handle. */
        simpleble_peripheral_release_handle(peripheral);
    }
    pthread_mutex_unlock(&ble_peripheral_mutex);

    /* Release all allocated memory. */
    simpleble_free(peripheral_identifier);
    simpleble_free(peripheral_address);
}

/*!
 * @brief API to track variables for writing chunked data
 */
static void track_write_data(size_t data_length)
{
    /* Track the actual write data length */
    actual_write_data_length += data_length;

    /* Check if all write data has been sent */
    if (actual_write_data_length == expected_write_data_length)
    {
        actual_write_data_length = 0;
        expected_write_data_length = 0;
        wait_for_tx_notify();
    }
}

/*!
 * @brief Writes `data_length` bytes of data from the provided `data` into the circular buffer.
 */
static void write_to_circular_buffer(uint8_t *data, uint8_t data_length)
{
    uint8_t i = 0;

    while (i < data_length)
    {
        /* Next write index position */
        uint16_t next_write_idx = (write_idx + 1) % COM_READ_BUFF;

        /* Check if the buffer is full */
        if (next_write_idx != read_idx)
        {
            device_response_buffer[write_idx] = data[i];
            write_idx = next_write_idx;
            ++i;
        }
        else
        {
            /* Buffer is full */
            break;
        }
    }
}

/*!
 * @brief Reads `n_bytes` of data from the circular buffer into the provided `buffer`
 */
static uint32_t read_from_circular_buffer(uint8_t *buffer, uint32_t n_bytes)
{
    uint32_t bytes_read = 0;

    while ((read_idx <= write_idx) && (bytes_read < n_bytes))
    {
        buffer[bytes_read] = device_response_buffer[read_idx];
        read_idx++;
        bytes_read++;
    }

    if (read_idx >= write_idx)
    {
        read_idx = 0;
        write_idx = 0;
    }

    return bytes_read;
}

/*!
 * @brief Returns available space in the circular buffer
 */
static uint32_t get_buffer_free_space(void)
{
    if (write_idx >= read_idx)
    {
        return write_idx - read_idx;
    }
    else
    {
        return (COM_READ_BUFF - read_idx) + write_idx;
    }
}

/*!
 * @brief callback function to store device response on TX notify
 */
static void peripheral_on_notify(simpleble_uuid_t service,
                                 simpleble_uuid_t characteristic,
                                 const uint8_t *data,
                                 size_t data_length,
                                 void* userdata)
{
    (void)service;
    (void)characteristic;
    (void)userdata;

    /* Mutex lock */
    pthread_mutex_lock(&ble_peripheral_mutex);

    /* Write to buffer */
    /*lint -e1773*/
    write_to_circular_buffer((uint8_t*) data, (uint8_t)data_length);
    /*lint +e1773*/
    has_tx_notified = true;

    /* Mutex unlock */
    pthread_mutex_unlock(&ble_peripheral_mutex);

    /*lint -e1746 parameter cannot be chaged to const*/
}

/*!
 * @brief Enables TX notify
 */
static int8_t ble_notify(void)
{
    /*lint -e605 */
    int8_t error_code = simpleble_peripheral_notify(selected_peripheral,
                                                    nordic_uart_service_uuid,
                                                    nordic_uart_char_tx,
                                                    peripheral_on_notify,
                                                    NULL);

    return error_code;
}

/*!
 * @brief Callback for Adapter scan start
 */
static void adapter_on_scan_start(simpleble_adapter_t adapter, void* userdata)
{
    (void)userdata;
    char *identifier;

    identifier = simpleble_adapter_identifier(adapter);

    if (identifier == NULL)
    {
        return;
    }

    printf("\nAdapter %s started scanning.\n", identifier);

    /* Clear the allocated memory. */
    simpleble_free(identifier);
}

/*!
 * @brief Callback for Adapter scan stop
 */
static void adapter_on_scan_stop(simpleble_adapter_t adapter, void* userdata)
{
    (void)userdata;
    char *identifier;

    identifier = simpleble_adapter_identifier(adapter);

    if (identifier == NULL)
    {
        return;
    }

    printf("\nAdapter %s stopped scanning.\n", identifier);

    /* Clear the allocated memory. */
    simpleble_free(identifier);
}

/*!
 * @brief Timeout for TX notify to happen for a write_request
 */
static void wait_for_tx_notify()
{
    time_t start = time(NULL);

    while (!has_tx_notified)
    {
        if (difftime(time(NULL), (long)start) >= BLE_TX_NOTIFY_TIMEOUT)
        {
            break;
        }

#ifdef PLATFORM_WINDOWS
        Sleep(1);
#else
        usleep(1);
#endif
    }
}

/*!
 * @brief Performs SimpleBLE write_request operation
 */
static int8_t ble_write_request(void *buffer, uint32_t n_bytes)
{

    int8_t error_code = simpleble_peripheral_write_request(selected_peripheral,
                                                           nordic_uart_service_uuid,
                                                           nordic_uart_char_rx,
                                                           (uint8_t*)buffer,
                                                           n_bytes);

    return error_code;
}

/*********************************************************************/
/* Export functions */
/*********************************************************************/

/*!
 * @brief Connects to BLE Adapter and returns list of BLE peripherals by initializing dll load
 */
int8_t ble_scan(struct ble_peripheral_info *ble_info, uint8_t *peripheral_count, size_t scan_timeout_ms)
{
    char *peripheral_identifier, *peripheral_address;
    int16_t peripheral_rssi;

#ifdef PLATFORM_WINDOWS
    if (!init_dll_loader())
    {
        ble_scan_result = BLE_COM_E_BLE_LIB_NOT_LOADED;

        return ble_scan_result;
    }

#endif

    /* Disable SimpleBLE library logs */
    simpleble_logging_set_level(SIMPLEBLE_LOG_LEVEL_NONE);

    size_t adapter_count = simpleble_adapter_get_count();
    if (adapter_count == 0)
    {
        ble_scan_result = BLE_COM_E_ADAPTOR_NOT_FOUND;

        return ble_scan_result;
    }

    /* TO DO: Automatically select host PC as adaptor */
    selected_adapter = simpleble_adapter_get_handle(0);
    if (selected_adapter == NULL)
    {
        ble_scan_result = BLE_COM_E_ADAPTOR_NOT_FOUND;

        return ble_scan_result;
    }

    if (!simpleble_adapter_is_bluetooth_enabled())
    {
        ble_scan_result = BLE_COM_E_ADAPTER_BLUETOOTH_NOT_ENABLED;

        return ble_scan_result;
    }

    (void)simpleble_adapter_set_callback_on_scan_start(selected_adapter, adapter_on_scan_start, NULL);
    (void)simpleble_adapter_set_callback_on_scan_stop(selected_adapter, adapter_on_scan_stop, NULL);
    (void)simpleble_adapter_set_callback_on_scan_found(selected_adapter, adapter_on_scan_found, NULL);

    if (scan_timeout_ms < MIN_SCAN_TIMEOUT_MS)
    {
        scan_timeout_ms = MIN_SCAN_TIMEOUT_MS;
    }

    (void)simpleble_adapter_scan_for(selected_adapter, (int)scan_timeout_ms);
    if (ble_peripheral_info_list[0].ble_peripheral == NULL && peripheral_list_len <= 1)
    {
        ble_scan_result = BLE_COM_E_PERIPHERAL_NOT_FOUND;

        return ble_scan_result;
    }

    /* Sort the BLE devices found based on rssi value */
    qsort(ble_peripheral_info_list, peripheral_list_len, sizeof(struct local_ble_peripheral_info), compare_rssi);

    printf("\nThe following BLE devices were found:\n");
    if (ble_info != NULL)
    {
        *peripheral_count = peripheral_list_len;
    }

    for (int8_t i = 0; i < peripheral_list_len; i++)
    {
        peripheral_identifier = ble_peripheral_info_list[i].ble_identifier;
        peripheral_address = ble_peripheral_info_list[i].ble_address;
        peripheral_rssi = ble_peripheral_info_list[i].ble_rssi;
        printf("[%d] %s [%s] [%d dBm]\n", i, peripheral_identifier, peripheral_address, peripheral_rssi);
        if (ble_info != NULL)
        {
            strcpy(ble_info[i].ble_identifier, peripheral_identifier);
            strcpy(ble_info[i].ble_address, peripheral_address);
        }
    }

    ble_scan_completed = true;

    return ble_scan_result;
}

/*!
 * @brief Establishes connection to BLE peripheral with peripheral index
 */
int8_t ble_connect(struct ble_peripheral_info *ble_config)
{
    int8_t error_code;
    int8_t ble_peripheral_index = -1;
    int8_t closest_app_board_index = -1;

    if (!ble_scan_completed && ble_scan_result == BLE_COM_OK)
    {
        error_code = ble_scan(NULL, NULL, 0);
        if (error_code != BLE_COM_OK)
        {
            return BLE_COM_E_PERIPHERAL_NOT_FOUND;
        }
    }
    else if (ble_scan_result != BLE_COM_OK)
    {
        return ble_scan_result;
    }

    strcpy(nordic_uart_service_uuid.value, NORDIC_UART_SERVICE_UUID);
    strcpy(nordic_uart_char_rx.value, NORDIC_UART_CHAR_RX);
    strcpy(nordic_uart_char_tx.value, NORDIC_UART_CHAR_TX);

    if (ble_config == NULL)
    {
        closest_app_board_index = get_ble_index_by_key(CLOSEST_APP_BOARD, NULL);
        if (closest_app_board_index != -1)
        {
            ble_peripheral_index = closest_app_board_index;
        }
        else
        {
            return BLE_COM_E_APP_BOARD_NOT_FOUND;
        }
    }
    else
    {
        if (strlen(ble_config->ble_identifier))
        {
            ble_peripheral_index = get_ble_index_by_key(IDENTIFIER, ble_config->ble_identifier);
        }

        if (ble_peripheral_index == -1)
        {
            ble_peripheral_index = get_ble_index_by_key(ADDRESS, ble_config->ble_address);
        }

        if (ble_peripheral_index == -1)
        {
            return BLE_COM_E_INVALID_COM_CONFIG;
        }
    }

    error_code = connect_to_ble_peripheral(ble_peripheral_index);
    if (error_code != BLE_COM_OK)
    {

        return BLE_COM_E_CONNECT_FAILED;
    }

    error_code = ble_notify();
    if (error_code != BLE_COM_OK)
    {
        return BLE_COM_E_TX_NOTIFY_FAILED;
    }

    return BLE_COM_OK;
}

/*!
 * @brief Performs write operation using BLE interface
 */
int8_t ble_write(void *buffer, uint32_t n_bytes)
{
    int8_t error_code;

    has_tx_notified = false;

    if (!is_ble_peripheral_connected)
    {
        return BLE_COM_E_PERIPHERAL_NOT_CONNECTED;
    }

    error_code = ble_write_request(buffer, n_bytes);

    if (error_code != BLE_COM_OK)
    {
        return BLE_COM_E_WRITE_FAILED;
    }

    if (((uint8_t*)buffer)[0] == COINES_WRITE_CMD)
    {
        memcpy(&expected_write_data_length, (uint8_t*)buffer + 1, 2); /* COINES_PROTO_LENGTH_POS index */
        is_write_data_chunked = (expected_write_data_length != n_bytes);
    }

    /* Handle data chunking */
    if (is_write_data_chunked)
    {
        track_write_data(n_bytes);
    }
    else
    {
        wait_for_tx_notify();
    }

    return BLE_COM_OK;
}

/*!
 * @brief Sets buffer with data received on BLE TX Notify
 */
int8_t ble_read(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read)
{
    uint32_t bytes_to_read = 0;
    uint32_t available_bytes;

    if (!is_ble_peripheral_connected)
    {
        return BLE_COM_E_PERIPHERAL_NOT_CONNECTED;
    }

    /* Mutex lock */
    pthread_mutex_lock(&ble_peripheral_mutex);

    /* If the requested data length is greater than available bytes then send available bytes */
    available_bytes = get_buffer_free_space();
    if (n_bytes > available_bytes)
    {
        bytes_to_read = available_bytes;
    }
    else
    {
        bytes_to_read = n_bytes;
    }

    /* Read from buffer */
    *n_bytes_read = read_from_circular_buffer((uint8_t*)buffer, bytes_to_read);

    /* Mutex unlock */
    pthread_mutex_unlock(&ble_peripheral_mutex);

    return BLE_COM_OK;
}

/*!
 * @brief Performs close operation of the BLE communication.
 *
 */
int8_t ble_close(void)
{
    // int8_t error_code = BLE_COM_OK; 

    // (void)simpleble_peripheral_unsubscribe(selected_peripheral, nordic_uart_service_uuid, nordic_uart_char_tx);
    // error_code = simpleble_peripheral_disconnect(selected_peripheral);
	is_ble_peripheral_connected = false;
	ble_scan_completed = false;
    clean_on_exit();
#ifdef PLATFORM_WINDOWS
    deinit_dll();
#endif

//     if (error_code != BLE_COM_OK)
//     {
//         clean_on_exit();
// #ifdef PLATFORM_WINDOWS
//         deinit_dll();
// #endif

//         return BLE_COM_E_DISCONNECT_FAILED;
//     }

    printf("\nBLE connection status: Disconnected\n");
    return BLE_COM_OK;
}
