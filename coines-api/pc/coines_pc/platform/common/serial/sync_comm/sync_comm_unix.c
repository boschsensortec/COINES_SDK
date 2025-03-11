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
 *  @file   sync_comm_unix.c
 *  @brief  This file contains the implementation of the synchronous serial communication functions for Linux and Mac
 *
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>

#include <../libusbp-1.0/includes/libusbp.h>
#include "platform.h"
#include "sync_comm_unix.h"
#include "error_handling.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static variables */
/*********************************************************************/
static int serial_handle;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static int8_t configure_port(uint32_t baudrate, char *port_name);

/*********************************************************************/
/* Functions */
/*********************************************************************/

/**
 * @brief Configures the serial port with the given baud rate and port name.
 *
 * @param baudrate The baud rate to configure the port with.
 * @param port_name The name of the port to configure.
 * @return Returns COINES_SUCCESS on success, or an error code on failure.
 */
static int8_t configure_port(uint32_t baudrate, char *port_name)
{
    int file_handle;

    file_handle = open(port_name, O_RDWR | O_NOCTTY);
    if (file_handle < 0)
    {
        return PLATFORM_SERIAL_PORT_IN_USE;
    }

    /* Flush away any bytes previously read or written. */
    tcflush(file_handle, TCIOFLUSH);

    /* Get the current configuration of the serial port. */
    struct termios options;
    if (tcgetattr(file_handle, &options) < 0)
    {
        close(file_handle);

        return PLATFORM_SERIAL_CONFIG_FAILED;
    }

    /*
     * Turn off any options that might interfere with our ability to send and
     * receive raw binary bytes.
     */
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    /* Configure timeout */
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    /* Configure baud rate */
    switch (baudrate)
    {
        case 4800:
            cfsetospeed(&options, B4800);
            break;
        case 9600:
            cfsetospeed(&options, B9600);
            break;
        case 19200:
            cfsetospeed(&options, B19200);
            break;
        case 38400:
            cfsetospeed(&options, B38400);
            break;
        case 115200:
            cfsetospeed(&options, B115200);
            break;
        default:
            cfsetospeed(&options, B115200);
            break;
    }
    cfsetispeed(&options, cfgetospeed(&options));

    if (tcsetattr(file_handle, TCSANOW, &options) < 0)
    {
        close(file_handle);

        return PLATFORM_SERIAL_CONFIG_FAILED;
    }

    serial_handle = file_handle;

    return COINES_SUCCESS;
}

/**
 * @brief Initializes the serial communication.
 *
 * @param baud_rate The baud rate to use for the communication.
 * @param vid The vendor ID of the device.
 * @param pid The product ID of the device.
 * @param port_name The name of the port to use for the communication.
 * @return Returns COINES_SUCCESS on success, or an error code on failure.
 */
int8_t serial_open(uint32_t baud_rate, uint16_t vid, uint16_t pid, char *port_name)
{
    int8_t ret = COINES_SUCCESS;
    libusbp_device **dev_list = NULL;
    libusbp_serial_port *port = NULL;
    size_t dev_count = 0;
    uint8_t index;
    libusbp_error *error;
    char *serial_port;
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;

    error = libusbp_list_connected_devices(&dev_list, &dev_count);
    (void)error;
    for (index = 0; index < dev_count; index++)
    {
        error = libusbp_device_get_vendor_id(dev_list[index], &vendor_id);
        (void)error;
        error = libusbp_device_get_product_id(dev_list[index], &product_id);
        (void)error;
        if ((vid == vendor_id) && (product_id == pid))
        {
            error = libusbp_serial_port_create(dev_list[index], 0, true, &port);
            if (error == NULL)
            {
                error = libusbp_serial_port_get_name(port, &serial_port);
                if ((port_name == NULL) || (strcmp(port_name, serial_port) == 0))
                {
                    ret = configure_port(baud_rate, serial_port);
                    libusbp_serial_port_free(port);
                    libusbp_string_free(serial_port);
                    if ((ret == COINES_SUCCESS) || (port_name != NULL && ret == PLATFORM_SERIAL_PORT_IN_USE))
                    {
                        break;
                    }
                }
            }
            else
            {
                ret = PLATFORM_SERIAL_DEVICE_NOT_SERIAL;
                break;
            }
        }
        else
        {
            ret = PLATFORM_SERIAL_DEV_NOT_FOUND;
        }
    }

    libusbp_list_free(dev_list);

    return ret;
}

/**
 * @brief Sends data over the serial communication.
 *
 * @param buffer The data to send.
 * @param n_bytes The number of bytes to send.
 * @return Returns COINES_SUCCESS on success, or an error code on failure.
 */
int8_t serial_write(void * buffer, uint32_t n_bytes)
{
    ssize_t bytes_written = 0;

    if (serial_handle)
    {
        bytes_written = write(serial_handle, buffer, n_bytes);
        if (bytes_written == (ssize_t)n_bytes)
        {
            return COINES_SUCCESS;
        }
        else
        {
            return PLATFORM_SERIAL_WRITE_FAILED;
        }
    }
    else
    {
        return PLATFORM_SERIAL_PORT_NOT_OPEN;
    }
}

/**
 * @brief Reads data from the serial communication.
 *
 * @param buffer The buffer to store the read data.
 * @param n_bytes The number of bytes to read.
 * @param n_bytes_read The number of bytes actually read.
 * @return Returns COINES_SUCCESS on success, or an error code on failure.
 */
int8_t serial_read(void * buffer, uint32_t n_bytes, uint32_t *n_bytes_read)
{
    ssize_t bytes_read = 0;

    if (serial_handle)
    {
        bytes_read = read(serial_handle, buffer, n_bytes);
        if (bytes_read < 0)
        {
            return PLATFORM_SERIAL_READ_FAILED;
        }

        *n_bytes_read = bytes_read;

        return COINES_SUCCESS;
    }
    else
    {
        return PLATFORM_SERIAL_PORT_NOT_OPEN;
    }
}

/**
 * @brief Closes the serial communication.
 *
 * @return Returns COINES_SUCCESS on success, or an error code on failure.
 */
int8_t serial_close(void)
{
    if (serial_handle)
    {
        close(serial_handle);
        serial_handle = 0;

        return COINES_SUCCESS;
    }
    else
    {
        return PLATFORM_SERIAL_PORT_NOT_OPEN;
    }
}

/**
 * @brief Clears the transmit and receive buffer.
 *
 * @return Returns COINES_SUCCESS on success, or an error code on failure.
 */
int8_t serial_clear_buffer(void)
{
    if (tcflush(serial_handle, TCIOFLUSH) == 0)
    {
        return COINES_SUCCESS;
    }

    return PLATFORM_SERIAL_FLUSH_FAILED;
}

/**
 * @brief Checks if the serial port is closed.
 *
 */
int8_t serial_is_port_closed(void)
{
    return COINES_E_FAILURE;  
}
