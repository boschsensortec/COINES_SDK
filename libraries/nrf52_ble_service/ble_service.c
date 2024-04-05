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

#include "ble_service.h"

#define ENVIRONMENTAL_SERVICE_UUID_BASE         {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
                                                      0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define ENVIRONMENTAL_SERVICE_UUID               0x181A
#define TEMPERATURE_CHAR                         0x2A6E
#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(12000)                 /**< Battery level measurement interval (ticks). This value corresponds to 12 seconds. */

APP_TIMER_DEF(m_battery_timer_id); /**< Battery measurement timer. */

extern bool ble_nus_connected;
extern bool ble_bas_connected;
volatile bool ble_tx_pending = false;
volatile size_t ble_nus_available = 0;
static char ble_device_name[32] = { 0 };
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can
 * be transmitted to the peer by the Nordic UART
 * service module. */
static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifier. */
{
  { BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE },
  { BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE },
  { ENVIRONMENTAL_SERVICE_UUID, BLE_UUID_TYPE_BLE }
};
ble_service_init_t service_init_info;
char ble_transmit_buff[BLE_GATT_WRITE_LEN];
FILE *bt_w;
FILE *bt_r;
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Environmental Sensing Service event type. */
typedef enum
{
    BLE_ESS_EVT_NOTIFICATION_ENABLED, /**<  notification enabled event. */
    BLE_ESS_EVT_NOTIFICATION_DISABLED /**< notification disabled event. */
} ble_ess_evt_type_t;

/**@brief Environmental sensing Service event. */
typedef struct
{
    ble_ess_evt_type_t evt_type; /**< Type of event. */
} ble_ess_evt_t;

// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;

/**@brief Environmental sensing Service event handler type. */
typedef void (*ble_ess_evt_handler_t)(ble_ess_t * p_bas, ble_ess_evt_t * p_evt);

/**@brief Environmental sensing init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ess_evt_handler_t evt_handler; /**< Event handler to be called for handling events in the Environmental sensing Service. */
    ble_srv_cccd_security_mode_t value_char_attr_md; /**< Initial security level for characteristics attribute */
} ble_ess_init_t;

/**@brief Environmental sensing Service structure. This contains various status information for the service. */
struct ble_ess_s
{
    ble_ess_evt_handler_t evt_handler; /**< Event handler to be called for handling events in the Environmental sensing Service. */
    uint16_t service_handle; /**< Handle of Environmental sensing Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t value_handles; /**< Handles related to the  Value characteristic. */
    uint16_t conn_handle; /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t uuid_type;
};

/**@brief   Macro for defining a custom service instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */

#define BLE_CUS_DEF(_name)                                                                          \
static ble_ess_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_NUS_BLE_OBSERVER_PRIO,                                                     \
                     ble_ess_on_ble_evt, &_name)

/*Environmental Sensing Service init instance*/
BLE_CUS_DEF(ble_temp_service);

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    service_init_info.batt_status_read_callback();
}
/**@brief Function for adding the temperature Value characteristic.
 *
 * @param[in]   p_ess        temp Service structure.
 * @param[in]   p_ess_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t temperature_value_char_add(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    uint32_t err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&char_md, 0, sizeof(char_md));

    //Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    char_md.char_props.read = 1;
    char_md.char_props.write = 0;
    char_md.char_props.notify = 0;
    char_md.char_props.indicate = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_ess->uuid_type;
    ble_uuid.uuid = TEMPERATURE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm = p_ess_init->value_char_attr_md.read_perm;
    attr_md.write_perm = p_ess_init->value_char_attr_md.write_perm;
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 1;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 8;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = 8;
    err_code = sd_ble_gatts_characteristic_add(p_ess->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ess->value_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for initializing the Environmental sensing Service.
 *
 * @param[out]  p_ess       Environmental sensing Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_ess_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
static uint32_t environment_sensing_service_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    if (p_ess == NULL || p_ess_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_ess->evt_handler = p_ess_init->evt_handler;
    p_ess->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add Enviornmental sensing Service UUID
    ble_uuid128_t base_uuid = { ENVIRONMENTAL_SERVICE_UUID_BASE };
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ess->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_ess->uuid_type;
    ble_uuid.uuid = ENVIRONMENTAL_SERVICE_UUID;

    // Add the Environmental sensing Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ess->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add temperature characteristic
    return temperature_value_char_add(p_ess, p_ess_init);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    if (NRF_SUCCESS == err_code)
        // Create battery timer.
        err_code = app_timer_create(&m_battery_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)service_init_info.adv_name,(uint16_t) strlen((const char *)service_init_info.adv_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */

/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    size_t length;

    switch (p_evt->type)
    {
        case BLE_NUS_EVT_RX_DATA:
            length = p_evt->params.rx_data.length;
            if (NRF_SUCCESS == nrf_ringbuf_cpy_put(&m_ringbuf, p_evt->params.rx_data.p_data, &length))
            {
                ble_nus_available += length;
            }
            break;
        case BLE_NUS_EVT_TX_RDY:
            ble_tx_pending = false;
            break;
        case BLE_NUS_EVT_COMM_STARTED:
            ble_nus_connected = true;
            break;
        case BLE_NUS_EVT_COMM_STOPPED:
            ble_nus_connected = false;
            break;
    }

}
/**@brief Function for handling the  Environmental Sensing Service events.
 *
 * @details This function will be called for all  Environmental Sensing Service events which are passed to
 *          the application.
 *
 * @param[in]   p_ess_service  Environmental Sensing Service structure.
 * @param[in]   p_evt          Event received from the Service.
 */
static void ble_ess_evt_handler(ble_ess_t * p_ess_service, ble_ess_evt_t * p_evt)
{
    (void)p_ess_service;
    switch (p_evt->evt_type)
    {
        case BLE_ESS_EVT_NOTIFICATION_ENABLED:
            break;

        case BLE_ESS_EVT_NOTIFICATION_DISABLED:
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the  battery Service events.
 *
 * @details This function will be called for all battery Service events which are passed to
 *          the application.
 *
 * @param[in]   p_ess_service  battery Service structure.
 * @param[in]   p_evt          Event received from the Service.
 */
static void ble_bas_evt_handler(ble_bas_t * p_bas_service, ble_bas_evt_t * p_evt)
{
    uint32_t err_code;
    (void)p_bas_service;
    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
            ble_bas_connected = true;
            service_init_info.batt_status_read_callback();
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            ble_bas_connected = false;
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_nus_init_t nus_init;
    ble_ess_init_t ble_service_ess_serv_init = { 0 };
    ble_bas_init_t bas_init = { 0 };
    nrf_ble_qwr_init_t qwr_init = { 0 };

    /* Initialize Queued Write Module. */
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    /* Initialize NUS. */
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize  Environmental Sensing Service init structure to zero.
    memset(&ble_service_ess_serv_init, 0, sizeof(ble_service_ess_serv_init));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ble_service_ess_serv_init.value_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ble_service_ess_serv_init.value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ble_service_ess_serv_init.value_char_attr_md.write_perm);

    ble_service_ess_serv_init.evt_handler = (ble_ess_evt_handler_t)ble_ess_evt_handler;
    err_code = environment_sensing_service_init(&ble_temp_service, &ble_service_ess_serv_init);
    APP_ERROR_CHECK(err_code);

    // Initialize battery Service init structure to zero.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler = (ble_bas_evt_handler_t)ble_bas_evt_handler;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        // commented the disconnect call to keep the connection alive
        //err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            break;
        default:
            break;
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_temp_char_write(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Check if the value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_ess->value_handles.cccd_handle)
        && (p_evt_write->len == 2)
        )
    {
        // CCCD written, call application event handler
        if (p_ess->evt_handler != NULL)
        {
            ble_ess_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_ess->evt_handler(p_ess, &evt);
        }
    }

}
/*
 * @breif encode floating value (temp_in_celcius) into the 32 bit single precision format
 */
static uint32_t temperature_encode(float data)
{
    int8_t exponent = -2;
    int32_t mantissa = (int32_t)(data * 100);
    uint32_t encoded_temp = ((exponent << 24) & 0xFF000000) | ((mantissa << 0) & 0x00FFFFFF);
    return encoded_temp;
}

static ret_code_t on_temp_char_read(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    if (p_ess == NULL || p_ble_evt == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    ret_code_t err_code;

    ble_gatts_evt_rw_authorize_request_t const * ar = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    memset(&auth_reply, 0, sizeof(auth_reply));

    float data;
    uint8_t len = 0;
    service_init_info.temp_read_callback(&data, &len);

    uint32_t encoded_temp = temperature_encode(data);

    if (ar->type == BLE_GATTS_AUTHORIZE_TYPE_READ)
    {
        /*reset the characteristics buffer before copying the new user data*/
        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
        auth_reply.params.read.len = len;
        auth_reply.params.read.p_data = (uint8_t*)&encoded_temp;
        auth_reply.params.read.update = 1;
        auth_reply.params.read.offset = ar->request.read.offset;
        auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;

        err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                   &auth_reply);

        VERIFY_SUCCESS(err_code);
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    (void)p_context;
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            /* LED indication will be changed when advertising starts. */
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            ble_nus_connected = false;
            ble_bas_connected = false;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            {
            ble_gap_phys_t const phys = {
                                          .rx_phys = BLE_GAP_PHY_AUTO,
                                          .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            /* Pairing not supported */
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            /* No system attributes have been stored. */
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            /* Disconnect on GATT Client timeout event. */
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            /* Disconnect on GATT Server timeout event. */
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            /* No implementation needed. */
            break;
    }
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Environmental Sensing Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 *
 * @param[in]   p_context   Environmental Sensing Service structure.
 */
static void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            ble_temp_service.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            ble_temp_service.conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
            on_temp_char_write(&ble_temp_service, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            if (TEMPERATURE_CHAR == p_ble_evt->evt.gatts_evt.params.authorize_request.request.read.uuid.uuid)
            {
                (void)on_temp_char_read(&ble_temp_service, p_ble_evt);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}
/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    /*
     * Configure the BLE stack using the default settings.
     * Fetch the start address of the application RAM.
     */
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    /* Enable BLE stack. */
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    /* Register a handler for BLE events. */
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    (void)p_gatt;
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = ((p_evt->params.att_mtu_effective - OPCODE_LENGTH) - HANDLE_LENGTH);
        (void)setvbuf(bt_w, ble_transmit_buff, _IOLBF, m_ble_nus_max_data_len);
    }
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}
/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, service_init_info.tx_power);
    APP_ERROR_CHECK(err_code);
}
void ble_service_init(ble_service_init_t* init_handle)
{
    service_init_info.temp_read_callback = init_handle->temp_read_callback;
    service_init_info.batt_status_read_callback = init_handle->batt_status_read_callback;
    service_init_info.adv_name = init_handle->adv_name;
    service_init_info.tx_power = init_handle->tx_power;

    /* Initialize. */
    timers_init();
    ble_stack_init();
    if (NULL == service_init_info.adv_name)
    {
        ble_gap_addr_t device_addr;
        if (NRF_SUCCESS == sd_ble_gap_addr_get(&device_addr))
        {
            sprintf(ble_device_name, "%s(%.2X-%.2X)", (const char*)BLE_DEVICE_NAME, device_addr.addr[1], device_addr.addr[0]);
            service_init_info.adv_name = ble_device_name;
        }
    }
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    advertising_start();
    tx_power_set();
    bt_w = fopen(BLE_DEVICE_FILE, "w");
    bt_r = fopen(BLE_DEVICE_FILE, "r");

    nrf_ringbuf_init(&m_ringbuf);

}

size_t ble_service_nus_write(const void *buffer, size_t len)
{
    uint32_t err_code;

    while (true)
    {
        err_code = ble_nus_data_send(&m_nus, (uint8_t *)buffer, (uint16_t *)&len, m_conn_handle);
        if(err_code == NRF_ERROR_RESOURCES)
        {
            coines_yield();
        }
        else
        {
            break;
        }
    }
    return len;
}

size_t ble_service_nus_read(void *buffer, size_t len)
{
    if (NRF_SUCCESS != nrf_ringbuf_cpy_get(&m_ringbuf, (uint8_t*)buffer, &len))
    {
        return 0;
    }
    if (len != 0)
    {
        ble_nus_available -= len;
        return len;
    }
    else
    {
        ble_nus_available = 0;
        return -1;
    }

}
int8_t ble_service_battery_level_update(uint8_t battery_level, uint8_t len)
{
    (void)len;
    ret_code_t err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if (err_code != NRF_SUCCESS)
    {
        return -1;
    }
    return 0;
}
