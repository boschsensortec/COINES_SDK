USE_FREERTOS=1
FREERTOS_CONFIG_PATH ?= 0

nRF5_SDK_DIR = $(COINES_INSTALL_PATH)/thirdparty/nRF5_SDK
THIRD_PARTY_DIR=$(COINES_INSTALL_PATH)/thirdparty
COINES_API_DIR=$(COINES_INSTALL_PATH)/coines-api
COMMON=$(COINES_INSTALL_PATH)/coines-api/common
LIB_DIR= $(COINES_INSTALL_PATH)/libraries
FREERTOS_PATH = $(nRF5_SDK_DIR)/external/freertos

ifeq ($(USE_FREERTOS),1)
ifeq ($(TARGET),MCU_APP30)
    APP30_SRC_DIR = $(COINES_INSTALL_PATH)/coines-api/mcu_app30
    include $(APP30_SRC_DIR)/mcu_app30.mk
    INCLUDEPATHS += $(INCLUDEPATHS_COINES)
endif

ifeq ($(TARGET),MCU_APP31)
    APP31_SRC_DIR = $(COINES_INSTALL_PATH)/coines-api/mcu_app31
    include $(APP31_SRC_DIR)/mcu_app31.mk
    INCLUDEPATHS += $(INCLUDEPATHS_COINES)
endif
endif


ifeq ($(USE_FREERTOS),1)
    USE_RTC_CLOCK=1
    CFLAGS +=-DAPP_TIMER_CONFIG_USE_SCHEDULER=1
# Convert error to warning for FREERTOS_CONFIG_PATH
    ifeq ($(FREERTOS_CONFIG_PATH),0)
        FREERTOS_CONFIG_PATH = $(COINES_INSTALL_PATH)/thirdparty/nRF5_SDK/external/freertos/config
    endif
endif


