#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <export.h>

#include <types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *
 * @return bool
 */
SIMPLEBLE_EXPORT bool simpleble_adapter_is_bluetooth_enabled(void);

/**
 * @brief
 *
 * @return size_t
 */
SIMPLEBLE_EXPORT size_t simpleble_adapter_get_count(void);

/**
 * @brief
 *
 * @note The user is responsible for freeing the returned adapter object
 *       by calling `simpleble_adapter_release_handle`.
 *
 * @param index
 * @return simpleble_adapter_t
 */
SIMPLEBLE_EXPORT simpleble_adapter_t simpleble_adapter_get_handle(size_t index);

/**
 * @brief Releases all memory and resources consumed by the specific
 *        instance of simpleble_adapter_t.
 *
 * @param handle
 */
SIMPLEBLE_EXPORT void simpleble_adapter_release_handle(simpleble_adapter_t handle);

/**
 * @brief Returns the identifier of a given adapter.
 *
 * @note The user is responsible for freeing the returned value.
 *
 * @param handle
 * @return char*
 */
SIMPLEBLE_EXPORT char* simpleble_adapter_identifier(simpleble_adapter_t handle);

/**
 * @brief Returns the MAC address of a given adapter.
 *
 * @note The user is responsible for freeing the returned value.
 *
 * @param handle
 * @return char*
 */
SIMPLEBLE_EXPORT char* simpleble_adapter_address(simpleble_adapter_t handle);

/**
 * @brief
 *
 * @param handle
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_scan_start(simpleble_adapter_t handle);

/**
 * @brief
 *
 * @param handle
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_scan_stop(simpleble_adapter_t handle);

/**
 * @brief
 *
 * @param handle
 * @param active
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_scan_is_active(simpleble_adapter_t handle, bool* active);

/**
 * @brief
 *
 * @param handle
 * @param timeout_ms
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_scan_for(simpleble_adapter_t handle, int timeout_ms);

/**
 * @brief
 *
 * @param handle
 * @return size_t
 */
SIMPLEBLE_EXPORT size_t simpleble_adapter_scan_get_results_count(simpleble_adapter_t handle);

/**
 * @brief
 *
 * @note The user is responsible for freeing the returned peripheral object
 *       by calling `simpleble_peripheral_release_handle`.
 *
 * @param handle
 * @param index
 * @return simpleble_peripheral_t
 */
SIMPLEBLE_EXPORT simpleble_peripheral_t simpleble_adapter_scan_get_results_handle(simpleble_adapter_t handle,
                                                                                  size_t index);

/**
 * @brief
 *
 * @param handle
 * @return size_t
 */
SIMPLEBLE_EXPORT size_t simpleble_adapter_get_paired_peripherals_count(simpleble_adapter_t handle);

/**
 * @brief
 *
 * @note The user is responsible for freeing the returned peripheral object
 *       by calling `simpleble_peripheral_release_handle`.
 *
 * @param handle
 * @param index
 * @return simpleble_peripheral_t
 */
SIMPLEBLE_EXPORT simpleble_peripheral_t simpleble_adapter_get_paired_peripherals_handle(simpleble_adapter_t handle,
                                                                                        size_t index);

/**
 * @brief
 *
 * @param handle
 * @param callback
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_set_callback_on_scan_start(
    simpleble_adapter_t handle, void (*callback)(simpleble_adapter_t adapter, void* userdata), void* userdata);

/**
 * @brief
 *
 * @param handle
 * @param callback
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_set_callback_on_scan_stop(
    simpleble_adapter_t handle, void (*callback)(simpleble_adapter_t adapter, void* userdata), void* userdata);

/**
 * @brief
 *
 * @param handle
 * @param callback
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_set_callback_on_scan_updated(
    simpleble_adapter_t handle,
    void (*callback)(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata), void* userdata);

/**
 * @brief
 *
 * @param handle
 * @param callback
 * @return simpleble_err_t
 */
SIMPLEBLE_EXPORT simpleble_err_t simpleble_adapter_set_callback_on_scan_found(
    simpleble_adapter_t handle,
    void (*callback)(simpleble_adapter_t adapter, simpleble_peripheral_t peripheral, void* userdata), void* userdata);

#ifdef __cplusplus
}
#endif
