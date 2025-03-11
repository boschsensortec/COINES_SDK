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
 *  @file   serial.c
 *  @brief  This module defines serial APIs to be used by the protocol layer
 *
 */

/**********************************************************************************/
/* system header files */
/**********************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "platform.h"
#include "coines.h"
#include "ble.h"
#include "circular_buffer.h"
#include "error_handling.h"
#include "api.h"

#ifdef PLATFORM_WINDOWS
#include "sync_comm_windows.h"
#else
#include "sync_comm_unix.h"
#endif

/*********************************************************************/
/* extern variables */
/*********************************************************************/
extern circular_buffer_t stream_cbuf;

/*********************************************************************/
/* static variables */
/*********************************************************************/
static pthread_t platform_serial_read_thread;
volatile bool is_read_thread_running = false;
static pthread_mutex_t serial_read_mutex = PTHREAD_MUTEX_INITIALIZER;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static void *platform_serial_read_thread_func(void *arg);
static void platform_ble_notify_callback(void *buffer, uint32_t length);



/*********************************************************************/
/* functions */
/*********************************************************************/
/**
 * @brief This API is used to continuously reads data from the serial port
 * 		  and stores the read data in a circular buffer.
 */
static void *platform_serial_read_thread_func(void *arg)
{
	(void)arg;
    uint32_t resp_length = 0;
	uint8_t read_buffer[SERIAL_READ_BUFF_SIZE];
	memset(read_buffer, 0, sizeof(read_buffer));
    while (1) 
	{
		pthread_mutex_lock(&serial_read_mutex);
        if (!is_read_thread_running) 
        {
            pthread_mutex_unlock(&serial_read_mutex);
            break;
        }
        pthread_mutex_unlock(&serial_read_mutex);

		resp_length = 0;

		if(circular_buffer_free_space(&stream_cbuf) > SERIAL_READ_BUFF_SIZE){
			if (serial_read(read_buffer, sizeof(read_buffer), &resp_length) != COINES_SUCCESS) 
			{
				pthread_mutex_lock(&serial_read_mutex);
				is_read_thread_running = false;
				pthread_mutex_unlock(&serial_read_mutex);
				break;
			}

			if(resp_length){
				(void)circular_buffer_put(&stream_cbuf, read_buffer, (uint16_t)resp_length);	
			}
		}
		else{
			while (!circular_buffer_is_empty(&stream_cbuf))
			{
				printf("Buffer full, emptying buffer\n");
				coines_delay_msec(10);
				break;
			}
		}
		coines_delay_usec(100);
	}
	pthread_exit(NULL);
	return NULL;
	
}

/**
 * @brief Callback function for BLE notifications. Populates circular buffer with received BLE data
 */
static void platform_ble_notify_callback(void *buffer, uint32_t length) {
    (void)circular_buffer_put(&stream_cbuf, (uint8_t *)buffer, (uint16_t)length);	
}

/**
 * @brief This API is used to start the serial read thread.
 */
int16_t platform_serial_read_thread_start(void)
{
	int16_t ret;
	is_read_thread_running = true;
	ret = pthread_create(&platform_serial_read_thread, NULL, platform_serial_read_thread_func, NULL);
	if ( ret != COINES_SUCCESS) 
	{
		return PLATFORM_THREAD_CREATE_FAILED;
	}
	return COINES_SUCCESS;
}

/**
 * @brief This API is used to stop the serial read thread.
 */
int16_t platform_serial_read_thread_stop(void)
{
	int16_t ret;
	
	pthread_mutex_lock(&serial_read_mutex);
	is_read_thread_running = false;
	pthread_mutex_unlock(&serial_read_mutex);
	ret = pthread_join(platform_serial_read_thread, NULL);
	
	if ( ret != COINES_SUCCESS)
	{
		return PLATFORM_THREAD_STOP_FAILED;
	}
	
	return COINES_SUCCESS;
}

/**
 * @brief This API is used to open the serial communication interface with the specified configuration.
 */
int16_t platform_open_serial(void *serial_config)
{
    int16_t ret;
	struct coines_serial_com_config* serial_com_config = (struct coines_serial_com_config*) serial_config;

    ret = serial_open(serial_com_config->baud_rate,
                       serial_com_config->vendor_id,
                       serial_com_config->product_id,
                       serial_com_config->com_port_name);
	return ret;
}

/**
 * @brief This API is used to close the serial communication.
 */
int16_t platform_close_serial(void)
{  
    return serial_close();
}

/**
 * @brief This API is used to send data over the serial communication platform.
 */
int16_t platform_send_serial(void *buffer, uint32_t n_bytes)
{
    return serial_write(buffer, n_bytes);
}

/**
 * @brief Sets the callback function for BLE notifications.
 */
void platform_ble_attach_notify_cb(bool attach_cb){
	if(attach_cb){
		ble_set_on_notify_callback(platform_ble_notify_callback);
	}else{
		ble_set_on_notify_callback(NULL);
	}
}

/**
 * @brief This API is used to receive data over the serial communication platform.
 */
int16_t platform_receive_serial(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read)
{
    return serial_read(buffer, n_bytes, n_bytes_read);
}

/*!
 * @brief This API is used to scan BLE devices
 */
int16_t platform_scan_ble(void *ble_info, uint8_t *peripheral_count, size_t scan_timeout_ms)
{

    struct ble_peripheral_info* info = (struct ble_peripheral_info*) ble_info;
    size_t timeout = (size_t)scan_timeout_ms;

    return ble_scan(info, peripheral_count, timeout);
}

/**
 * @brief This API is used to open the ble communication interface with the specified configuration.
 */
int16_t platform_open_ble(void *ble_config)
{
    return ble_connect((struct ble_peripheral_info *)ble_config);
}

/**
 * @brief This API is used to close the ble communication platform.
 */
int16_t platform_close_ble(void)
{
    return ble_close();
}

/**
 * @brief This API is used to send data over the BLE communication platform.
 */
int16_t platform_send_ble(void *buffer, uint32_t n_bytes)
{
    return ble_write(buffer, n_bytes);
}

/**
 * @brief This API is used to receive data over the BLE communication platform.
 */
int16_t platform_receive_ble(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read)
{
    return ble_read(buffer, n_bytes, n_bytes_read);
}

/*!
 * @brief This API is used to flush the buffer
 *
 */
void platform_flush_serial(void)
{
    (void)serial_clear_buffer();
}

/*!
 * @brief Checks the status of the serial port.
 *
 */
int16_t platform_check_serial_port_status(void)
{
	return serial_is_port_closed();
}

/*!
 * @brief Checks the status of the ble.
 *
 */
int16_t platform_check_ble_status(void)
{
	return ble_is_disconnected();
}

