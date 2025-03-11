COINES_INSTALL_PATH = ../../../../../

# C_SRCS += main.c
# Application source file
EXAMPLE_FILE ?= main.c


# Update FREERTOS_CONFIG_PATH to use the current directory
FREERTOS_CONFIG_PATH = $(CURDIR)/config

TARGET=MCU_APP30
LOCATION = FLASH

INCLUDEPATHS += $(FREERTOS_CONFIG_PATH)
include $(COINES_INSTALL_PATH)/libraries/freeRTOS/freertos_dependencies.mk

include $(COINES_INSTALL_PATH)/coines.mk