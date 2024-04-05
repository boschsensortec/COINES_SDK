
/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file        app_config.h
 *
 * @brief
 *
 */

// <e> NRFX_SAADC_ENABLED - nrfx_saadc - SAADC peripheral driver
//==========================================================
#ifndef NRFX_SAADC_ENABLED
#define NRFX_SAADC_ENABLED 1
#endif
// <o> NRFX_SAADC_CONFIG_RESOLUTION  - Resolution

// <0=> 8 bit
// <1=> 10 bit
// <2=> 12 bit
// <3=> 14 bit

#ifndef NRFX_SAADC_CONFIG_RESOLUTION
#define NRFX_SAADC_CONFIG_RESOLUTION 1
#endif

// <o> NRFX_SAADC_CONFIG_OVERSAMPLE  - Sample period

// <0=> Disabled
// <1=> 2x
// <2=> 4x
// <3=> 8x
// <4=> 16x
// <5=> 32x
// <6=> 64x
// <7=> 128x
// <8=> 256x

#ifndef NRFX_SAADC_CONFIG_OVERSAMPLE
#define NRFX_SAADC_CONFIG_OVERSAMPLE 0
#endif

// <q> NRFX_SAADC_CONFIG_LP_MODE  - Enabling low power mode


#ifndef NRFX_SAADC_CONFIG_LP_MODE
#define NRFX_SAADC_CONFIG_LP_MODE 0
#endif

// <o> NRFX_SAADC_CONFIG_IRQ_PRIORITY  - Interrupt priority

// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
// <4=> 4
// <5=> 5
// <6=> 6
// <7=> 7

#ifndef NRFX_SAADC_CONFIG_IRQ_PRIORITY
#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 6
#endif


// <e> NRFX_TWIM_ENABLED - nrfx_twim - TWIM peripheral driver
//==========================================================
#ifndef NRFX_TWIM_ENABLED
#define NRFX_TWIM_ENABLED 1
#endif
// <q> NRFX_TWIM0_ENABLED  - Enable TWIM0 instance
 

#ifndef NRFX_TWIM0_ENABLED
#define NRFX_TWIM0_ENABLED 1
#endif

// <q> NRFX_TWIM1_ENABLED  - Enable TWIM1 instance
 

#ifndef NRFX_TWIM1_ENABLED
#define NRFX_TWIM1_ENABLED 1
#endif

// <o> NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY  - Frequency
 
// <26738688=> 100k 
// <67108864=> 250k 
// <104857600=> 400k 

#ifndef NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY
#define NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY 26738688
#endif

// <q> NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT  - Enables bus holding after uninit
 

#ifndef NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT
#define NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT 1
#endif

// <o> NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

#ifndef NRFX_SPIM_ENABLED
#define NRFX_SPIM_ENABLED 1
#endif
// <q> NRFX_SPIM0_ENABLED  - Enable SPIM0 instance


#ifndef NRFX_SPIM0_ENABLED
#define NRFX_SPIM0_ENABLED 1
#endif

// <q> NRFX_SPIM1_ENABLED  - Enable SPIM1 instance

#ifndef NRFX_SPIM1_ENABLED
#define NRFX_SPIM1_ENABLED 0
#endif

// <q> NRFX_SPIM2_ENABLED  - Enable SPIM2 instance

#ifndef NRFX_SPIM2_ENABLED
#define NRFX_SPIM2_ENABLED 1
#endif

// <q> NRFX_SPIM3_ENABLED  - Enable SPIM3 instance

#ifndef NRFX_SPIM3_ENABLED
#define NRFX_SPIM3_ENABLED 1
#endif

// <q> NRFX_SPIM_EXTENDED_ENABLED  - Enable extended SPIM features


#ifndef NRFX_SPIM_EXTENDED_ENABLED
#define NRFX_SPIM_EXTENDED_ENABLED 0
#endif

// <o> NRFX_SPIM_MISO_PULL_CFG  - MISO pin pull configuration.

// <0=> NRF_GPIO_PIN_NOPULL
// <1=> NRF_GPIO_PIN_PULLDOWN
// <3=> NRF_GPIO_PIN_PULLUP

#ifndef NRFX_SPIM_MISO_PULL_CFG
#define NRFX_SPIM_MISO_PULL_CFG 1
#endif

// <o> NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority

// <0=> 0 (highest)
// <1=> 1
// <2=> 2
// <3=> 3
// <4=> 4
// <5=> 5
// <6=> 6
// <7=> 7

#ifndef NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

#define NRF_SDH_ENABLED  1
#define NRF_SDH_CLOCK_LF_SRC 1
#define NRF_SDH_CLOCK_LF_RC_CTIV 0
#define NRF_SDH_CLOCK_LF_RC_TEMP_CTIV 0
#define NRF_SDH_CLOCK_LF_ACCURACY 7

#define NRF_SDH_BLE_ENABLED 1
#define NRF_SDH_BLE_GAP_DATA_LENGTH 251
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 1
#define NRF_SDH_BLE_CENTRAL_LINK_COUNT 0
#define NRF_SDH_BLE_TOTAL_LINK_COUNT 1
#define NRF_SDH_BLE_GAP_EVENT_LENGTH 6
#define NRF_SDH_BLE_GATT_MAX_MTU_SIZE 247
#define NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE 1408
#define NRF_SDH_BLE_VS_UUID_COUNT 2
#define NRF_SDH_BLE_SERVICE_CHANGED 0
#define NRF_SDH_BLE_STACK_OBSERVER_PRIO 0

#define NRF_SDH_SOC_STACK_OBSERVER_PRIO 0
#define NRF_SDH_STACK_OBSERVER_PRIO_LEVELS 2
#define NRF_SECTION_ITER_ENABLED 1
#define NRF_SDH_SOC_ENABLED 1
#define CLOCK_CONFIG_SOC_OBSERVER_PRIO 0
#define NRF_SDH_SOC_OBSERVER_PRIO_LEVELS 2
#define CLOCK_CONFIG_STATE_OBSERVER_PRIO 0
#define NRF_SDH_STATE_OBSERVER_PRIO_LEVELS 2

#define POWER_CONFIG_SOC_OBSERVER_PRIO 0
#define POWER_CONFIG_STATE_OBSERVER_PRIO 0

#define NRF_SDH_BLE_OBSERVER_PRIO_LEVELS 4

#define NRF_BLE_GATT_ENABLED 1
#define NRF_BLE_GATT_BLE_OBSERVER_PRIO 1
#define NRF_BLE_CONN_PARAMS_ENABLED 1

#define BLE_NUS_ENABLED 1
#define BLE_NUS_BLE_OBSERVER_PRIO 2
#define BLE_CONN_STATE_BLE_OBSERVER_PRIO 0

#define BLE_BAS_ENABLED 1
#define BLE_BAS_BLE_OBSERVER_PRIO 2

#define NRF_BLE_QWR_ENABLED 1
#define NRF_BLE_QWR_BLE_OBSERVER_PRIO 2

#define NRF_BLE_CONN_PARAMS_ENABLED 1
#define NRF_BLE_CONN_PARAMS_MAX_SLAVE_LATENCY_DEVIATION 499
#define NRF_BLE_CONN_PARAMS_MAX_SUPERVISION_TIMEOUT_DEVIATION 65535
#define BLE_CONN_PARAMS_BLE_OBSERVER_PRIO 1

#define BLE_ADVERTISING_ENABLED 1
#define BLE_ADV_BLE_OBSERVER_PRIO 1
#define BLE_ADV_SOC_OBSERVER_PRIO 1

