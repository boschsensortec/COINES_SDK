/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * Copyright (c) 2023, Bosch Sensortec GmbH
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef NRF52_BLE_SERVICE_H_
#define NRF52_BLE_SERVICE_H_

#include <stdio.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus.h"
#include "ble_bas.h"
#include <string.h>
#include "ble_advdata.h"
#include "nrf_ble_gatt.h"
#include "app_error.h"
#include "stdint.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "ble_advertising.h"
#include "nrf_ringbuf.h"

#ifdef MCU_APP30
#define BLE_DEVICE_NAME                 "APP Board 3.0"/**<if user has not configured ble name using coines_ble_config() then
                                                         default ble name will be used along with last four digit of mac address (APP Board 3.0(XX-XX)) */
#endif

#ifdef MCU_NICLA
#define BLE_DEVICE_NAME                 "Nicla Sense ME "
#endif

#ifdef MCU_APP31
#define BLE_DEVICE_NAME                 "APP Board 3.1 "
#endif

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the
                                                                                     * SoftDevice BLE configuration. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART
                                                                                     * Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer
                                                                                     * priority. You shouldn't need to
                                                                                     * modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in
                                                                                     * units of 0.625 ms. This value
                                                                                     * corresponds to 40 ms). */

#define APP_ADV_DURATION                0                                           /**< The advertising duration 0
                                                                                     * means enabling unlimited advertising(no timeout)
                                                                                     */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)             /**< Minimum acceptable connection
                                                                                     * interval (15 ms), Connection
                                                                                     * interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Maximum acceptable connection
                                                                                     * interval (37.5 ms), Connection
                                                                                     * interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout
                                                                                     * (4 seconds), Supervision Timeout
                                                                                     * uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event
                                                                                     * (connect or start of
                                                                                     * notification) to first time
                                                                                     * sd_ble_gap_conn_param_update is
                                                                                     * called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to
                                                                                     * sd_ble_gap_conn_param_update
                                                                                     * after the first call (30
                                                                                     * seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before
                                                                                     * giving up the connection
                                                                                     * parameter negotiation. */
//lint -e506 -e514 -e762
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr); /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
BLE_BAS_DEF(m_bas); /**< BLE battery service instance */

#define BLE_GATT_WRITE_LEN              256
#define RECEIVE_BUFF_LEN                256
#define BLE_NUS_FD_W                    8
#define BLE_NUS_FD_R                    9
#define BLE_DEVICE_FILE                 "/dev/ble"

NRF_RINGBUF_DEF(m_ringbuf, RECEIVE_BUFF_LEN);

typedef void (*temp_read_callback)(void * data, uint8_t* data_len);
typedef void (*bat_read_callback)(void);

typedef struct ble_service_init_s
{
    temp_read_callback temp_read_callback;
    bat_read_callback batt_status_read_callback;
    char * adv_name;
    int8_t tx_power;
}ble_service_init_t;

void ble_service_init(ble_service_init_t* init_handle);
size_t ble_service_nus_write(const void *buffer, size_t len);
size_t ble_service_nus_read(void *buffer, size_t len);
int8_t ble_service_battery_level_update(uint8_t battery_level,uint8_t len);
void coines_yield(void);


#endif /* NRF52_BLE_SERVICE_H_ */
