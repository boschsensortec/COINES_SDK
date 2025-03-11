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
 * @file    usb.c
 * @brief   This file contains USB module related API definitions
 *
 */

/*!
 * @defgroup usb_api usb
 * @{*/

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#ifdef PLATFORM_WINDOWS
#include <windows.h>
#endif

#ifdef PLATFORM_LINUX
#include <pthread.h>
#include <unistd.h>
#endif

#ifdef LEGACY_USB_DRIVER
#include "legacy_usb_support.h"
#endif

#if defined(LIBUSB_DRIVER) && defined(PLATFORM_WINDOWS)
#include "libusb.h"
#endif

#if defined(LIBUSB_DRIVER) && defined(PLATFORM_LINUX)
#include <libusb-1.0/libusb.h>
#endif

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "usb.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/

/*! USB Bulk end point out */
#define USB_BULK_EP_OUT             UINT8_C(0x02)

/*! USB Bulk end point in */
#define USB_BULK_EP_IN              UINT8_C(0x81)

/*! USB data read out time out */
#define USB_TIMEOUT                 INT16_C(3000)

/*! USB packet size*/
#define USB_PACKET_SIZE             64

/*! USB Number of transfers to submit */
#define USB_NO_TRANSFERS_TO_SUBMIT  UINT8_C(3)

/*********************************************************************/
/* global variables */
/*********************************************************************/
/*! variable to hold the different board types*/
int usb_board_type = 0;

/*! variable to hold the usb channel status*/
uint8_t usb_channel_inprogress = 1;

/*!
 * @brief Variable used to hold the thread reference which reads the data
 * from communication layer.
 */
#ifdef PLATFORM_WINDOWS
HANDLE usb_keep_alive_thread;
DWORD usb_keep_alive_id;
#endif
#ifdef PLATFORM_LINUX
pthread_t usb_keep_alive_thread;
pthread_attr_t usb_keep_alive_attr;
struct sched_param usb_keep_alive_sched_param;
#endif

/*! USB response buffer */
usb_rsp_buffer_t usb_rsp_buf[USB_NO_TRANSFERS_TO_SUBMIT];

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 * @brief This function is used to keep USB alive
 */
#ifdef PLATFORM_WINDOWS
DWORD WINAPI usb_keep_alive(void * arg);

#endif
#ifdef PLATFORM_LINUX
static void *usb_keep_alive(void *arg);

#endif

#ifdef LIBUSB_DRIVER

/*!
 * @brief This internal callback function triggered for USB in events .
 */
static void usb_transfer_event_callback(struct libusb_transfer *transfer);

/*!
 * @brief This function is used to find the usb device
 */
libusb_device * usb_find_device(libusb_device **devices);

/*!
 * @brief This internal callback function triggered for USB async response event
 */
libusb_transfer_cb_fn usb_call_back;

/*! libusb transfer structure for IN request */
struct libusb_transfer *usb_transfer_handle[USB_NO_TRANSFERS_TO_SUBMIT] = { 0 };

/*! Variable holds the USB handle. */
libusb_device_handle *usb_handle;

/*! Variable holds the USB context */
libusb_context *usb_ctx;

/*! Variable holds the USB interface number */
uint8_t interfaceNumber = 0;
#endif

/*!
 * @brief This internal callback function triggered for USB async response event
 */
usb_async_response_call_back usb_rsp_callback;

/*********************************************************************/
/* static variables */
/*********************************************************************/
/*! USB init status */
static uint8_t usb_initialized = 0;

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 * @brief This API is used to establish the LIB USB communication.
 */
int16_t usb_open_device(coines_command_t* comm_buf, usb_async_response_call_back rsp_cb)
{
#ifdef LIBUSB_DRIVER
    libusb_device **device_list;
    uint8_t count;
#endif
    if ((comm_buf == NULL) || (rsp_cb == NULL))
    {

        /* Null pointer error */
        return COINES_E_NULL_PTR;
    }

#ifdef LIBUSB_DRIVER
    if (libusb_init(&usb_ctx) < 0)
    {
        return COINES_E_COMM_IO_ERROR;
    }

    /*Get the device List*/
    if (libusb_get_device_list(usb_ctx, &device_list) < 0)
    {
        libusb_exit(usb_ctx);

        return COINES_E_DEVICE_NOT_FOUND;
    }

    /* Find the USB device using PID and VID as defined */
    libusb_device *device = usb_find_device(device_list);
    if (device == NULL)
    {
        libusb_free_device_list(device_list, 1);
        libusb_exit(usb_ctx);

        return COINES_E_DEVICE_NOT_FOUND;
    }

    /*open usb device*/
    if (libusb_open(device, &usb_handle) < 0)
    {
        libusb_free_device_list(device_list, 1);
        libusb_exit(usb_ctx);

        return COINES_E_UNABLE_OPEN_DEVICE;
    }

#ifdef PLATFORM_LINUX
    libusb_detach_kernel_driver(usb_handle, 0);
#endif

    libusb_free_device_list(device_list, 1);

    if (libusb_claim_interface(usb_handle, interfaceNumber) < 0)
    {
        libusb_exit(usb_ctx);

        return COINES_E_UNABLE_CLAIM_INTF;
    }

#endif

    /*allocate memory for communication buffer*/
    memset(comm_buf, 0, sizeof(sizeof(coines_command_t)));
    comm_buf->board_type = (coines_board_t)usb_board_type;

    usb_rsp_callback = rsp_cb;
#ifdef LIBUSB_DRIVER
    for (count = 0; count < USB_NO_TRANSFERS_TO_SUBMIT; count++)
    {
        usb_transfer_handle[count] = libusb_alloc_transfer(0);

        libusb_fill_bulk_transfer(usb_transfer_handle[count],
                                  usb_handle,
                                  USB_BULK_EP_IN,
                                  (unsigned char *)(usb_rsp_buf[count].buffer),
                                  COINES_DATA_BUF_SIZE,
                                  (libusb_transfer_cb_fn)usb_transfer_event_callback,
                                  NULL,
                                  0);
    }

    if (libusb_submit_transfer(usb_transfer_handle[0]) < 0)
    {
        libusb_free_transfer(usb_transfer_handle[0]);

        return COINES_E_FAILURE;
    }

#endif
    usb_channel_inprogress = 1;

    usb_initialized = 1;

#ifdef PLATFORM_LINUX
    pthread_attr_init(&usb_keep_alive_attr);
    pthread_attr_setschedpolicy(&usb_keep_alive_attr, SCHED_FIFO);
    usb_keep_alive_sched_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&usb_keep_alive_attr, &usb_keep_alive_sched_param);
    pthread_create(&usb_keep_alive_thread, &usb_keep_alive_attr, usb_keep_alive, NULL);
#endif
#ifdef PLATFORM_WINDOWS

    /*Set priority class*/
    SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);

    /*set the priority of main thread*/

    usb_keep_alive_thread = CreateThread(NULL, /* default security attributes */
                                         0, /* use default stack size */
                                         usb_keep_alive, /* thread function */
                                         NULL, /* argument to thread function */
                                         0, /* use default creation flags */
                                         &usb_keep_alive_id); /* returns the thread identifier */

    SetThreadPriority(usb_keep_alive_thread, THREAD_PRIORITY_TIME_CRITICAL);

#endif

#ifdef LIBUSB_DRIVER

    return COINES_SUCCESS;
#endif

#ifdef LEGACY_USB_DRIVER
    int rslt;
    rslt = legacy_open_usb_connection();

    return rslt;
#endif
}

/*!
 * @brief This internal callback function triggered for USB in events .
 *
 * @param[in] transfer : transfer structure for USB IN events.
 *
 * @return None
 * @retval None
 *
 */
#if LIBUSB_DRIVER
static void usb_transfer_event_callback(struct libusb_transfer *transfer)
{
    int8_t count = 0;
    int8_t next = 0;

    switch (transfer->status)
    {
        case LIBUSB_TRANSFER_COMPLETED:

            if (usb_channel_inprogress <= USB_NO_TRANSFERS_TO_SUBMIT)
            {
                count = (int8_t)(usb_channel_inprogress - 1);

                if (transfer->actual_length > 0)
                {
                    usb_rsp_buf[count].buffer_size = transfer->actual_length;
                    usb_rsp_callback(&usb_rsp_buf[count]);

                    /*Prepare the next available buffer for receiving usb data */
                    if (count == (USB_NO_TRANSFERS_TO_SUBMIT - 1))
                    {
                        next = 0;

                        /* Reseting the next channel number to 1 if the number of channels count reached max */
                        memset(&usb_rsp_buf[next], 0, (size_t)transfer->actual_length);
                        usb_channel_inprogress = 1;
                    }
                    else
                    {
                        next = count + 1;
                        memset(&usb_rsp_buf[next], 0, (size_t)transfer->actual_length);
                        usb_channel_inprogress++;
                    }
                }

                if (usb_initialized)
                {
                    /* submit for receiving the next usb data*/
                    libusb_submit_transfer(usb_transfer_handle[next]);
                }
                else
                {
                    /*cancel transfer*//* TBD */
                    libusb_cancel_transfer(usb_transfer_handle[count]);
                }
            }

            break;
        case LIBUSB_TRANSFER_CANCELLED:
        case LIBUSB_TRANSFER_NO_DEVICE:
        case LIBUSB_TRANSFER_TIMED_OUT:
        case LIBUSB_TRANSFER_ERROR:
        case LIBUSB_TRANSFER_STALL:
        case LIBUSB_TRANSFER_OVERFLOW:

            /* Various type of errors here */
            break;
    }
}
#endif

/*!
 * @brief This internal callback function used to keep USB alive .
 *
 * @param[in] arg : void pointer.
 *
 * @return None
 * @retval None
 *
 */
#ifdef PLATFORM_WINDOWS
DWORD WINAPI usb_keep_alive(void * arg)
#endif
#ifdef PLATFORM_LINUX
static void *usb_keep_alive(void *arg)
#endif
{
    (void)arg;
#ifdef LEGACY_USB_DRIVER
    int16_t rslt = COINES_E_FAILURE;
#endif
    while (usb_initialized)
    {
#ifdef LIBUSB_DRIVER

        /*usb handle events are triggered to keep alive the libusb asynchronous communication*/
        if (libusb_handle_events(NULL) != LIBUSB_SUCCESS)
        {
            break;
        }

#endif
#ifdef LEGACY_USB_DRIVER
        rslt = legacy_read_usb_response(usb_rsp_buf[0].buffer);
        if (rslt == COINES_SUCCESS)
        {
            usb_rsp_callback(&usb_rsp_buf[0]);
            memset(&usb_rsp_buf[0].buffer, 0, COINES_DATA_BUF_SIZE);
        }

#endif
    }

#ifdef PLATFORM_WINDOWS

    return COINES_SUCCESS;
#endif

#ifdef PLATFORM_LINUX

    return NULL;
#endif
}
#ifdef LIBUSB_DRIVER

/*!
 * @brief This function used to find the USB device
 *
 * @param[in] devices : pointer to USB device
 *
 * @return status of the device
 *
 */

libusb_device * usb_find_device(libusb_device **devices)
{
    libusb_device *dev;
    int32_t idx = 0;
    int rslt;
    struct libusb_device_descriptor desc;
    bool dev_found = false;

    if (devices == NULL)
    {
        return NULL;
    }

    while ((dev = devices[idx++]) != NULL)
    {
        rslt = libusb_get_device_descriptor(dev, &desc);
        if (rslt >= 0)
        {
            /* Check for DD2.0 FW VID & PID */
            if ((desc.idProduct == COINES_DEVICE_DD_PRODUCT && desc.idVendor == COINES_DEVICE_DD_VENDOR) ||
                (desc.idProduct == COINES_WINUSB_APP30_PID && desc.idVendor == COINES_ROBERT_BOSCH_VID))
            {
                usb_board_type = COINES_BOARD_DD;
                interfaceNumber = 0;
                dev_found = true;
            }
        }

        if (dev_found == true && libusb_open(dev, &usb_handle) == 0)
        {

            #ifdef PLATFORM_LINUX
            libusb_detach_kernel_driver(usb_handle, 0);
            #endif

            /* Try accessing device */
            if (libusb_claim_interface(usb_handle, interfaceNumber) == 0)
            {
                libusb_release_interface(usb_handle, interfaceNumber);
                libusb_close(usb_handle);
                usb_handle = NULL;

                return dev;
            }
        }
    }

    return NULL;
}
#endif

/*!
 *  @brief This API closes the USB connection
 */
void usb_close_device(void)
{
    usb_initialized = 0;
#ifdef PLATFORM_WINDOWS

    /*Wait until all threads have terminated*/
    /*WaitForMultipleObjects(1, &usb_keep_alive_thread, TRUE, INFINITE); */
    CloseHandle(usb_keep_alive_thread);
#endif
#ifdef PLATFORM_LINUX
    pthread_attr_destroy(&usb_keep_alive_attr);

    /*pthread_join(usb_keep_alive_thread, NULL); */
    pthread_detach(usb_keep_alive_thread);
#endif
#ifdef LIBUSB_DRIVER
    if (usb_handle)
    {
        libusb_release_interface(usb_handle, 0);
#ifdef PLATFORM_LINUX
        libusb_attach_kernel_driver(usb_handle, 0);
#endif
        libusb_close(usb_handle);
        usb_handle = NULL;
    }

    libusb_exit(usb_ctx);
#endif

#ifdef LEGACY_USB_DRIVER
    legacy_close_usb_device();
#endif
}

/*!
 *  @brief This API is used to send the data to the board through USB.
 *
 */
int16_t usb_send_command(coines_command_t* buffer)
{
#ifdef LIBUSB_DRIVER
    uint32_t buffer_size; /**< buffer size */
#endif
    if (buffer == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    memset(&buffer->buffer[0] + buffer->buffer_size, 0, COINES_DATA_BUF_SIZE - buffer->buffer_size); /* variable buffer
                                                                                                      * size */

    /*TODO: debug code */
#if COINES_DEBUG_ENABLED == 1
    uint16_t i = 0;
    printf("index: %d - ", buffer->buffer_size);
    for (; i < buffer->buffer_size; i++)
    {
        printf("%x :", buffer->buffer[i]);
    }

    printf("\n");
#endif

#ifdef LIBUSB_DRIVER
    if (usb_handle == NULL)
    {
        return COINES_E_COMM_IO_ERROR;
    }

    int size;

    /* Making buffer size as multiple of 64 bytes(USB endpoint size) */
    buffer_size = buffer->buffer_size + (USB_PACKET_SIZE - (buffer->buffer_size % USB_PACKET_SIZE));
    buffer->error =(uint8_t) libusb_bulk_transfer(usb_handle,
                                         USB_BULK_EP_OUT,
                                         &buffer->buffer[0],
                                         (int)buffer_size,
                                         &size,
                                         USB_TIMEOUT);

    if (buffer->error == 0)
    {
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_FAILURE;
    }

#endif

#ifdef LEGACY_USB_DRIVER
    buffer->error = legacy_send_usb_command(&buffer->buffer[0], USB_PACKET_SIZE);
    if (buffer->error == TRUE)
    {
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_FAILURE;
    }

#endif
}

/** @}*/
