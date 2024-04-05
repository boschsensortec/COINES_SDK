/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file   simpleble_lib_loader.c
 * @brief  This module provides SimpleBLE function handles and dll load functions
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <windows.h>

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
#if defined(WIN64) || defined(_WIN64)
#define DLL  "simpleble-c.dll"
#else
#define DLL  "simpleble-c32.dll"
#endif

/*********************************************************************/
/* own header files */
/**********************************************************************/
#include "simpleble_lib_loader.h"

/*********************************************************************/
/* static variables */
/*********************************************************************/

HINSTANCE lib_instance;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

static bool load_library(void);
static void *get_address(const char *function_name);
static void free_library(void);

/*********************************************************************/
/* typedef declarations */
/*********************************************************************/
typedef void (*simpleble_free_t)(void *);
typedef size_t (*simpleble_adapter_get_count_t)(void);
typedef bool (*simpleble_adapter_is_bluetooth_enabled_t)(void);
typedef simpleble_adapter_t (*simpleble_adapter_get_handle_t)(size_t);
typedef void (*simpleble_adapter_release_handle_t)(simpleble_adapter_t);
typedef simpleble_err_t (*simpleble_adapter_scan_for_t)(simpleble_adapter_t, int);
typedef simpleble_err_t (*simpleble_adapter_scan_start_t)(simpleble_adapter_t);
typedef simpleble_err_t (*simpleble_adapter_scan_stop_t)(simpleble_adapter_t);
typedef simpleble_err_t (*simpleble_adapter_set_callback_on_scan_updated_t)(simpleble_adapter_t,
                                                                            void (*)(simpleble_adapter_t,
                                                                                     simpleble_peripheral_t, void *),
                                                                            void *);
typedef simpleble_err_t (*simpleble_adapter_set_callback_on_scan_start_t)(simpleble_adapter_t,
                                                                          void (*)(simpleble_adapter_t, void*), void*);
typedef simpleble_err_t (*simpleble_adapter_set_callback_on_scan_stop_t)(simpleble_adapter_t,
                                                                         void (*)(simpleble_adapter_t, void *), void*);
typedef simpleble_err_t (*simpleble_adapter_set_callback_on_scan_found_t)(simpleble_adapter_t,
                                                                          void (*)(simpleble_adapter_t,
                                                                                   simpleble_peripheral_t, void *),
                                                                          void *);
typedef char *(*simpleble_peripheral_address_t)(simpleble_peripheral_t);
typedef char *(*simpleble_peripheral_identifier_t)(simpleble_peripheral_t);
typedef char *(*simpleble_adapter_identifier_t)(simpleble_peripheral_t);
typedef simpleble_err_t (*simpleble_peripheral_connect_t)(simpleble_peripheral_t);
typedef simpleble_err_t (*simpleble_peripheral_disconnect_t)(simpleble_peripheral_t);
typedef void (*simpleble_peripheral_release_handle_t)(simpleble_peripheral_t);
typedef size_t (*simpleble_peripheral_services_count_t)(simpleble_peripheral_t);
typedef simpleble_err_t (*simpleble_peripheral_services_get_t)(simpleble_peripheral_t, size_t, simpleble_service_t*);
typedef simpleble_err_t (*simpleble_peripheral_write_request_t)(simpleble_peripheral_t, simpleble_uuid_t,
                                                                simpleble_uuid_t, uint8_t *, size_t);
typedef simpleble_err_t (*simpleble_peripheral_write_command_t)(simpleble_peripheral_t, simpleble_uuid_t,
                                                                simpleble_uuid_t, uint8_t*, size_t);
typedef simpleble_err_t (*simpleble_peripheral_notify_t)(simpleble_peripheral_t, simpleble_uuid_t, simpleble_uuid_t,
                                                         void (*)(simpleble_uuid_t, simpleble_uuid_t, const uint8_t *, size_t,
                                                                  void *), void *);
typedef simpleble_err_t (*simpleble_peripheral_unsubscribe_t)(simpleble_peripheral_t, simpleble_uuid_t,
                                                              simpleble_uuid_t);
typedef size_t (*simpleble_peripheral_manufacturer_data_count_t)(simpleble_peripheral_t);
typedef simpleble_err_t (*simpleble_peripheral_manufacturer_data_get_t)(simpleble_peripheral_t, size_t,
                                                                        simpleble_manufacturer_data_t*);
typedef simpleble_err_t (*simpleble_peripheral_set_callback_on_connected_t)(simpleble_peripheral_t,
                                                                            void (*)(simpleble_peripheral_t, void *),
                                                                            void *);
typedef simpleble_err_t (*simpleble_peripheral_set_callback_on_disconnected_t)(simpleble_peripheral_t,
                                                                               void (*)(simpleble_peripheral_t, void *),
                                                                               void *);
typedef int16_t (*simpleble_peripheral_rssi_t)(simpleble_peripheral_t);
typedef uint16_t (*simpleble_peripheral_mtu_t)(simpleble_peripheral_t);
typedef void (*simpleble_logging_set_level_t)(simpleble_log_level_t);
typedef simpleble_err_t (*simpleble_peripheral_is_connected_t)(simpleble_peripheral_t, bool*);

/*********************************************************************/
/* local functions */
/*********************************************************************/

/*!
 * @brief Load DLL library
 */
static bool load_library()
{
    if (lib_instance == NULL)
    {
        lib_instance = LoadLibrary(TEXT(DLL));
        if (lib_instance == NULL)
        {
            return false;
        }
    }

    return true;
}

/*!
 * @brief Gets address of the function from dll
 */
static void *get_address(const char *function_name)
{
    if (lib_instance == NULL)
    {
        return NULL;
    }

    /*lint -e{611} Suppress Suspicious cast*/
    return (void *)GetProcAddress(lib_instance, function_name);
}

/*!
 * @brief Free the loaded dll instance
 */
static void free_library()
{
    if (lib_instance)
    {
        (void)FreeLibrary(lib_instance);
        lib_instance = NULL;
    }
}

/*********************************************************************/
/* export functions */
/*********************************************************************/

/*!
 * @brief Frames dll path and loads the simpleble dll
 */
bool init_dll_loader(void)
{
    if (lib_instance == NULL)
    {
        if (!load_library())
        {
            lib_instance = NULL;
        }
    }

    return lib_instance != NULL;
}

/*!
 * @brief Free the loaded dll
 */
void deinit_dll(void)
{
    free_library();
}

/*********************************************************************/
/* export simpleble library functions */
/*********************************************************************/
void simpleble_free(void *handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_free_t func = (simpleble_free_t)get_address("simpleble_free");

    if (func == NULL)
    {
        return;
    }

    return func(handle);
}

size_t simpleble_adapter_get_count(void)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_get_count_t func = (simpleble_adapter_get_count_t)get_address("simpleble_adapter_get_count");

    if (func == NULL)
    {
        return 0;
    }

    return func();
}

bool simpleble_adapter_is_bluetooth_enabled(void)
{

    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_is_bluetooth_enabled_t func = (simpleble_adapter_is_bluetooth_enabled_t)get_address(
        "simpleble_adapter_is_bluetooth_enabled");

    if (func == NULL)
    {
        return false;
    }

    return func();
}

simpleble_adapter_t simpleble_adapter_get_handle(size_t adapter_index)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_get_handle_t func = (simpleble_adapter_get_handle_t)get_address("simpleble_adapter_get_handle");

    if (func == NULL)
    {
        return NULL;
    }

    return func(adapter_index);
}

void simpleble_adapter_release_handle(simpleble_adapter_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_release_handle_t func = (simpleble_adapter_release_handle_t)get_address(
        "simpleble_adapter_release_handle");

    if (func == NULL)
    {
        return;
    }

    func(handle);
}

simpleble_err_t simpleble_adapter_scan_for(simpleble_adapter_t handle, int timeout_ms)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_scan_for_t func = (simpleble_adapter_scan_for_t)get_address("simpleble_adapter_scan_for");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, timeout_ms);
}

simpleble_err_t simpleble_adapter_scan_start(simpleble_adapter_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_scan_start_t func = (simpleble_adapter_scan_start_t)get_address("simpleble_adapter_scan_start");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle);
}

simpleble_err_t simpleble_adapter_scan_stop(simpleble_adapter_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_scan_stop_t func = (simpleble_adapter_scan_stop_t)get_address("simpleble_adapter_scan_stop");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle);
}

simpleble_err_t simpleble_adapter_set_callback_on_scan_updated(simpleble_adapter_t handle, void (*callback)(
                                                                   simpleble_adapter_t adapter,
                                                                   simpleble_peripheral_t peripheral,
                                                                   void *userdata), void *userdata)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_set_callback_on_scan_updated_t func =
        (simpleble_adapter_set_callback_on_scan_updated_t)get_address("simpleble_adapter_set_callback_on_scan_updated");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, callback, userdata);
}

simpleble_err_t simpleble_adapter_set_callback_on_scan_start(simpleble_adapter_t handle, void (*callback)(
                                                                 simpleble_adapter_t,
                                                                 void *), void *userdata)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_set_callback_on_scan_start_t func = (simpleble_adapter_set_callback_on_scan_start_t)get_address(
        "simpleble_adapter_set_callback_on_scan_start");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, callback, userdata);
}

simpleble_err_t simpleble_adapter_set_callback_on_scan_stop(simpleble_adapter_t handle, void (*callback)(
                                                                simpleble_adapter_t,
                                                                void *), void *userdata)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_set_callback_on_scan_stop_t func = (simpleble_adapter_set_callback_on_scan_stop_t)get_address(
        "simpleble_adapter_set_callback_on_scan_stop");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, callback, userdata);
}

simpleble_err_t simpleble_adapter_set_callback_on_scan_found(simpleble_adapter_t handle, void (*callback)(
                                                                 simpleble_adapter_t adapter,
                                                                 simpleble_peripheral_t peripheral,
                                                                 void *userdata), void *userdata)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_set_callback_on_scan_found_t func = (simpleble_adapter_set_callback_on_scan_found_t)get_address(
        "simpleble_adapter_set_callback_on_scan_found");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, callback, userdata);
}

char *simpleble_peripheral_address(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_address_t func = (simpleble_peripheral_address_t)get_address("simpleble_peripheral_address");

    if (func == NULL)
    {
        return NULL;
    }

    return func(handle);
}

char *simpleble_peripheral_identifier(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_identifier_t func = (simpleble_peripheral_identifier_t)get_address(
        "simpleble_peripheral_identifier");

    if (func == NULL)
    {
        return NULL;
    }

    return func(handle);
}

char *simpleble_adapter_identifier(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_adapter_identifier_t func = (simpleble_adapter_identifier_t)get_address("simpleble_adapter_identifier");

    if (func == NULL)
    {
        return NULL;
    }

    return func(handle);
}

simpleble_err_t simpleble_peripheral_connect(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_connect_t func = (simpleble_peripheral_connect_t)get_address("simpleble_peripheral_connect");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle);
}

simpleble_err_t simpleble_peripheral_disconnect(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_disconnect_t func = (simpleble_peripheral_disconnect_t)get_address(
        "simpleble_peripheral_disconnect");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle);
}

void simpleble_peripheral_release_handle(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_release_handle_t func = (simpleble_peripheral_release_handle_t)get_address(
        "simpleble_peripheral_release_handle");

    if (func == NULL)
    {
        return;
    }

    func(handle);
}

size_t simpleble_peripheral_services_count(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_services_count_t func = (simpleble_peripheral_services_count_t)get_address(
        "simpleble_peripheral_services_count");

    if (func == NULL)
    {
        return 0;
    }

    return func(handle);
}

simpleble_err_t simpleble_peripheral_services_get(simpleble_peripheral_t handle,
                                                  size_t peripheral_index,
                                                  simpleble_service_t *services)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_services_get_t func = (simpleble_peripheral_services_get_t)get_address(
        "simpleble_peripheral_services_get");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, peripheral_index, services);
}

simpleble_err_t simpleble_peripheral_write_request(simpleble_peripheral_t handle,
                                                   simpleble_uuid_t service,
                                                   simpleble_uuid_t characteristic,
                                                   uint8_t *data,
                                                   size_t data_length)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_write_request_t func = (simpleble_peripheral_write_request_t)get_address(
        "simpleble_peripheral_write_request");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, service, characteristic, data, data_length);
}

simpleble_err_t simpleble_peripheral_write_command(simpleble_peripheral_t handle,
                                                   simpleble_uuid_t service,
                                                   simpleble_uuid_t characteristic,
                                                   uint8_t *data,
                                                   size_t data_length)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_write_command_t func = (simpleble_peripheral_write_command_t)get_address(
        "simpleble_peripheral_write_command");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, service, characteristic, data, data_length);
}

simpleble_err_t simpleble_peripheral_notify(simpleble_peripheral_t handle,
                                            simpleble_uuid_t service,
                                            simpleble_uuid_t characteristic,
                                            void (*callback)(simpleble_uuid_t service,
                                                             simpleble_uuid_t characteristic,
                                                             const uint8_t *data,
                                                             size_t data_length,
                                                             void *userdata),
                                            void *userdata)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_notify_t func = (simpleble_peripheral_notify_t)get_address("simpleble_peripheral_notify");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, service, characteristic, callback, userdata);
}

simpleble_err_t simpleble_peripheral_unsubscribe(simpleble_peripheral_t handle,
                                                 simpleble_uuid_t service,
                                                 simpleble_uuid_t characteristic)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_unsubscribe_t func = (simpleble_peripheral_unsubscribe_t)get_address(
        "simpleble_peripheral_unsubscribe");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, service, characteristic);
}

size_t simpleble_peripheral_manufacturer_data_count(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_manufacturer_data_count_t func = (simpleble_peripheral_manufacturer_data_count_t)get_address(
        "simpleble_peripheral_manufacturer_data_count");

    if (func == NULL)
    {
        return 0;
    }

    return func(handle);
}

simpleble_err_t simpleble_peripheral_manufacturer_data_get(simpleble_peripheral_t handle,
                                                           size_t peripheral_index,
                                                           simpleble_manufacturer_data_t *manufacturer_data)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_manufacturer_data_get_t func = (simpleble_peripheral_manufacturer_data_get_t)get_address(
        "simpleble_peripheral_manufacturer_data_get");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, peripheral_index, manufacturer_data);
}

simpleble_err_t simpleble_peripheral_is_connected(simpleble_peripheral_t handle, bool *connected)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_is_connected_t func = (simpleble_peripheral_is_connected_t)get_address(
        "simpleble_peripheral_is_connected");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, connected);
}

simpleble_err_t simpleble_peripheral_set_callback_on_connected(simpleble_peripheral_t handle, void (*callback)(
                                                                   simpleble_peripheral_t peripheral,
                                                                   void *userdata), void *userdata)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_set_callback_on_connected_t func =
        (simpleble_peripheral_set_callback_on_connected_t)get_address("simpleble_peripheral_set_callback_on_connected");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, callback, userdata);
}

simpleble_err_t simpleble_peripheral_set_callback_on_disconnected(simpleble_peripheral_t handle, void (*callback)(
                                                                      simpleble_peripheral_t peripheral,
                                                                      void *userdata), void *userdata)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_set_callback_on_disconnected_t func =
        (simpleble_peripheral_set_callback_on_disconnected_t)get_address(
            "simpleble_peripheral_set_callback_on_disconnected");

    if (func == NULL)
    {
        return SIMPLEBLE_FAILURE;
    }

    return func(handle, callback, userdata);
}

int16_t simpleble_peripheral_rssi(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_rssi_t func = (simpleble_peripheral_rssi_t)get_address("simpleble_peripheral_rssi");

    if (func == NULL)
    {
        return 0;
    }

    return func(handle);

}

uint16_t simpleble_peripheral_mtu(simpleble_peripheral_t handle)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_peripheral_mtu_t func = (simpleble_peripheral_mtu_t)get_address("simpleble_peripheral_mtu");

    if (func == NULL)
    {
        return 0;
    }

    return func(handle);

}

void simpleble_logging_set_level(simpleble_log_level_t level)
{
    /*lint -e{611} Suppress Suspicious cast*/
    simpleble_logging_set_level_t func = (simpleble_logging_set_level_t)get_address("simpleble_logging_set_level");

    if (func == NULL)
    {
        return;
    }

    return func(level);

}
