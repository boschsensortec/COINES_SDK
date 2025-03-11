nRF5_SDK_DIR = ../../thirdparty/nRF5_SDK
THIRD_PARTY_DIR=../../thirdparty
LIB_DIR=../../libraries
COMMON=../common
COINES_API_DIR=.


CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

CFLAGS += -std=c99 -mthumb -mabi=aapcs -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -c -Os -g -Wall
CFLAGS += -ffunction-sections -fdata-sections

CFLAGS += -DNRF52832_XXAA -D__HEAP_SIZE=8192 -D__STACK_SIZE=8192 -DSWI_DISABLE0 -DUSE_APP_CONFIG
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DS132
CFLAGS += -DSOFTDEVICE_PRESENT

nRF5_SDK_DIR = ../../thirdparty/nRF5_SDK
THIRD_PARTY_DIR=../../thirdparty
LIB_DIR=../../libraries
ASM_SRCS_COINES = $(nRF5_SDK_DIR)/modules/nrfx/mdk/gcc_startup_nrf52.S \

C_SRCS_COINES += \
../coines_common.c \
mcu_nicla_support.c \
mcu_nicla_interface.c \
mcu_nicla.c \
$(nRF5_SDK_DIR)/modules/nrfx/mdk/system_nrf52.c \
$(nRF5_SDK_DIR)/integration/nrfx/legacy/nrf_drv_clock.c \
$(nRF5_SDK_DIR)/integration/nrfx/legacy/nrf_drv_power.c \
$(nRF5_SDK_DIR)/integration/nrfx/legacy/nrf_drv_twi.c \
$(nRF5_SDK_DIR)/integration/nrfx/legacy/nrf_drv_spi.c \
$(nRF5_SDK_DIR)/integration/nrfx/legacy/nrf_drv_uart.c \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_rtc.c \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_clock.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_gpiote.c \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_power.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_power_clock.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_spim.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_wdt.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_systick.c \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_timer.c \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_twim.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_uarte.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/nrfx_uart.c  \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/src/prs/nrfx_saadc.c \
$(nRF5_SDK_DIR)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
$(nRF5_SDK_DIR)/components/libraries/util/app_util_platform.c \
$(nRF5_SDK_DIR)/components/libraries/util/app_error.c \
$(nRF5_SDK_DIR)/components/libraries/util/app_error_handler_gcc.c \
$(nRF5_SDK_DIR)/components/libraries/util/app_error_weak.c \
$(nRF5_SDK_DIR)/components/libraries/hardfault/nrf52/handler/hardfault_handler_gcc.c \
$(nRF5_SDK_DIR)/components/libraries/hardfault/hardfault_implementation.c \
$(nRF5_SDK_DIR)/components/libraries/atomic_fifo/nrf_atfifo.c \
$(nRF5_SDK_DIR)/components/libraries/atomic/nrf_atomic.c \
$(nRF5_SDK_DIR)/components/libraries/fifo/app_fifo.c \
$(nRF5_SDK_DIR)/components/libraries/uart/app_uart_fifo.c \
$(nRF5_SDK_DIR)/components/softdevice/common/nrf_sdh.c \
$(nRF5_SDK_DIR)/components/softdevice/common/nrf_sdh_ble.c \
$(nRF5_SDK_DIR)/components/softdevice/common/nrf_sdh_soc.c \
$(nRF5_SDK_DIR)/components/ble/common/ble_advdata.c \
$(nRF5_SDK_DIR)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
$(nRF5_SDK_DIR)/components/libraries/experimental_section_vars/nrf_section_iter.c \
$(nRF5_SDK_DIR)/external/segger_rtt/SEGGER_RTT.c \
$(nRF5_SDK_DIR)/external/segger_rtt/SEGGER_RTT_printf.c \
$(LIB_DIR)/nrf52_ble_service/ble_service.c \
$(LIB_DIR)/nicla_sense_led/led.c \
$(LIB_DIR)/pmic_bq25120/bq25120.c \
$(LIB_DIR)/i2c_common/i2c_common.c \
$(LIB_DIR)/nicla_sense_ext_flash/mx25_ext_flash.c \
$(LIB_DIR)/uart_common/uart_common.c \
$(THIRD_PARTY_DIR)/LittleFS/lfs_util.c \
$(THIRD_PARTY_DIR)/LittleFS/lfs.c \
$(nRF5_SDK_DIR)/components/ble/ble_services/ble_nus/ble_nus.c \
$(nRF5_SDK_DIR)/components/ble/ble_services/ble_bas/ble_bas.c \
$(nRF5_SDK_DIR)/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c \
$(nRF5_SDK_DIR)/components/ble/common/ble_conn_state.c \
$(nRF5_SDK_DIR)/components/ble/common/ble_srv_common.c \
$(nRF5_SDK_DIR)/components/ble/common/ble_conn_params.c \
$(nRF5_SDK_DIR)/components/libraries/atomic_flags/nrf_atflags.c \
$(nRF5_SDK_DIR)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c \
$(nRF5_SDK_DIR)/components/libraries/timer/app_timer.c \
$(nRF5_SDK_DIR)/components/ble/ble_advertising/ble_advertising.c \
$(nRF5_SDK_DIR)/components/libraries/ringbuf/nrf_ringbuf.c \

INCLUDEPATHS_COINES += \
.. \
. \
conf \
$(nRF5_SDK_DIR)/modules/nrfx \
$(nRF5_SDK_DIR)/modules/nrfx/mdk \
$(nRF5_SDK_DIR)/modules/nrfx/drivers/include \
$(nRF5_SDK_DIR)/modules/nrfx/hal \
$(nRF5_SDK_DIR)/components \
$(nRF5_SDK_DIR)/components/libraries/scheduler \
$(nRF5_SDK_DIR)/components/libraries/queue \
$(nRF5_SDK_DIR)/components/libraries/pwr_mgmt \
$(nRF5_SDK_DIR)/components/libraries/fifo \
$(nRF5_SDK_DIR)/components/libraries/strerror \
$(nRF5_SDK_DIR)/components/toolchain/cmsis/include \
$(nRF5_SDK_DIR)/components/libraries/timer \
$(nRF5_SDK_DIR)/components/libraries/util \
$(nRF5_SDK_DIR)/components/libraries/ringbuf \
$(nRF5_SDK_DIR)/components/libraries/hardfault/nrf52 \
$(nRF5_SDK_DIR)/components/libraries/hardfault \
$(nRF5_SDK_DIR)/components/libraries/log \
$(nRF5_SDK_DIR)/components/libraries/log/src \
$(nRF5_SDK_DIR)/components/libraries/experimental_section_vars \
$(nRF5_SDK_DIR)/components/libraries/mutex \
$(nRF5_SDK_DIR)/components/libraries/delay \
$(nRF5_SDK_DIR)/components/libraries/atomic_fifo \
$(nRF5_SDK_DIR)/components/libraries/atomic \
$(nRF5_SDK_DIR)/components/libraries/uart \
$(nRF5_SDK_DIR)/components/boards \
$(nRF5_SDK_DIR)/integration/nrfx \
$(nRF5_SDK_DIR)/integration/nrfx/legacy \
$(nRF5_SDK_DIR)/external/fnmatch \
$(nRF5_SDK_DIR)/external/utf_converter \
$(nRF5_SDK_DIR)/components/softdevice/s132/headers \
$(nRF5_SDK_DIR)/components/softdevice/s132/headers/nrf52 \
$(nRF5_SDK_DIR)/components/ble/common \
$(nRF5_SDK_DIR)/components/ble/nrf_ble_gatt \
$(nRF5_SDK_DIR)/components/softdevice/common \
$(nRF5_SDK_DIR)/external/segger_rtt \
$(LIB_DIR)/nrf52_ble_service/ \
$(LIB_DIR)/nicla_sense_led/ \
$(LIB_DIR)/pmic_bq25120/ \
$(LIB_DIR)/i2c_common/ \
$(LIB_DIR)/nicla_sense_pmic/ \
$(LIB_DIR)/nicla_sense_common/ \
$(LIB_DIR)/nicla_sense_ext_flash/ \
$(LIB_DIR)/uart_common/ \
$(THIRD_PARTY_DIR)/LittleFS/ \
$(nRF5_SDK_DIR)/components/ble/ble_services/ble_nus \
$(nRF5_SDK_DIR)/components/ble/ble_services/ble_bas \
$(nRF5_SDK_DIR)/components/ble/ble_link_ctx_manager \
$(nRF5_SDK_DIR)/components/libraries/atomic_flags \
$(nRF5_SDK_DIR)/components/ble/nrf_ble_qwr \
$(nRF5_SDK_DIR)/components/ble/ble_advertising \