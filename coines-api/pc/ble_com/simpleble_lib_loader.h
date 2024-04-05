/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file   simpleble_lib_loader.c
 * @brief  This file contains SimpleBLE and dll load related variable and function declarations
 *
 *
 */

#ifndef SIMPLEBLE_LIB_LOADER_H
#define SIMPLEBLE_LIB_LOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* Other header includes */
/**********************************************************************************/
#include "simpleble_types.h"

/**********************************************************************************/
/* (extern) variable declarations */
/**********************************************************************************/
extern HINSTANCE lib_instance;

/**********************************************************************************/
/* DLL loading related function declarations */
/**********************************************************************************/

/*!
 * @brief Frames dll path and loads the simpleble dll
 *
 * @return Result of API execution status.
 * @retval true -> Success.
 * @retval false -> Fail.
 */
bool init_dll_loader(void);

/*!
 * @brief Free the loaded dll
 *
 * @return void
 */
void deinit_dll(void);

/**********************************************************************************/
/* Simple function declarations */
/**********************************************************************************/
/* common */
void simpleble_free(void *handle);

/* adapter */
size_t simpleble_adapter_get_count(void);
bool simpleble_adapter_is_bluetooth_enabled(void);
simpleble_adapter_t simpleble_adapter_get_handle(size_t index);
void simpleble_adapter_release_handle(simpleble_adapter_t handle);
simpleble_err_t simpleble_adapter_scan_for(simpleble_adapter_t handle, int timeout_ms);
simpleble_err_t simpleble_adapter_scan_start(simpleble_adapter_t handle);
simpleble_err_t simpleble_adapter_scan_stop(simpleble_adapter_t handle);

simpleble_err_t simpleble_adapter_set_callback_on_scan_start(simpleble_adapter_t handle, void (*)(simpleble_adapter_t,
                                                                                                  void *), void *);
simpleble_err_t simpleble_adapter_set_callback_on_scan_stop(simpleble_adapter_t handle, void (*)(simpleble_adapter_t,
                                                                                                 void *), void *);
simpleble_err_t simpleble_adapter_set_callback_on_scan_updated(simpleble_adapter_t handle, void (*)(simpleble_adapter_t,
                                                                                                    simpleble_peripheral_t,
                                                                                                    void *), void *);
simpleble_err_t simpleble_adapter_set_callback_on_scan_found(simpleble_adapter_t handle, void (*)(simpleble_adapter_t,
                                                                                                  simpleble_peripheral_t,
                                                                                                  void *), void *);

/* peripheral */
char *simpleble_peripheral_address(simpleble_peripheral_t handle);
char *simpleble_peripheral_identifier(simpleble_peripheral_t handle);
char *simpleble_adapter_identifier(simpleble_peripheral_t handle);
void simpleble_peripheral_release_handle(simpleble_peripheral_t handle);
int16_t simpleble_peripheral_rssi(simpleble_peripheral_t handle);
uint16_t simpleble_peripheral_mtu(simpleble_peripheral_t handle);
simpleble_err_t simpleble_peripheral_connect(simpleble_peripheral_t handle);
simpleble_err_t simpleble_peripheral_disconnect(simpleble_peripheral_t handle);
size_t simpleble_peripheral_services_count(simpleble_peripheral_t handle);
simpleble_err_t simpleble_peripheral_services_get(simpleble_peripheral_t handle,
                                                  size_t index,
                                                  simpleble_service_t *services);
simpleble_err_t simpleble_peripheral_write_request(simpleble_peripheral_t handle,
                                                   simpleble_uuid_t service,
                                                   simpleble_uuid_t characteristic,
                                                   uint8_t *data,
                                                   size_t data_length);
simpleble_err_t simpleble_peripheral_write_command(simpleble_peripheral_t handle,
                                                   simpleble_uuid_t service,
                                                   simpleble_uuid_t characteristic,
                                                   uint8_t *data,
                                                   size_t data_length);

simpleble_err_t simpleble_peripheral_notify(simpleble_peripheral_t handle,
                                            simpleble_uuid_t service,
                                            simpleble_uuid_t characteristic,
                                            void (*)(simpleble_uuid_t, simpleble_uuid_t, const uint8_t *, size_t, void *),
                                            void *);
simpleble_err_t simpleble_peripheral_unsubscribe(simpleble_peripheral_t handle,
                                                 simpleble_uuid_t service,
                                                 simpleble_uuid_t characteristic);
size_t simpleble_peripheral_manufacturer_data_count(simpleble_peripheral_t handle);
simpleble_err_t simpleble_peripheral_manufacturer_data_get(simpleble_peripheral_t handle,
                                                           size_t index,
                                                           simpleble_manufacturer_data_t *manufacturer_data);
simpleble_err_t simpleble_peripheral_is_connected(simpleble_peripheral_t handle, bool *connected);
simpleble_err_t simpleble_peripheral_set_callback_on_connected(simpleble_peripheral_t handle, void (*callback)(
                                                                   simpleble_peripheral_t peripheral,
                                                                   void *userdata), void *userdata);
simpleble_err_t simpleble_peripheral_set_callback_on_disconnected(simpleble_peripheral_t handle, void (*callback)(
                                                                      simpleble_peripheral_t peripheral,
                                                                      void *userdata), void *userdata);
void simpleble_logging_set_level(simpleble_log_level_t level);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLE_LIB_LOADER_H */

/** @}*/
