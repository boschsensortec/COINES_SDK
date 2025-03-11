ifeq ($(OS),Windows_NT)
    PLATFORM = PLATFORM_WINDOWS
	DRIVER ?= LEGACY_USB_DRIVER
else
    PLATFORM = PLATFORM_LINUX
	DRIVER = LIBUSB_DRIVER
endif

CC     = gcc


CBUFF_SIZE ?= 104857600 # 100MB
MQUEUE_DEPTH ?= 100
MQUEUE_PACKET_SIZE ?=255
STREAM_RSP_TIMEOUT ?= 0 #Millisecond
FIFO_STREAM_RSP_TIMEOUT ?= 0 #Millisecond

CFLAGS += -DCIRCULAR_BUFFER_SIZE=$(CBUFF_SIZE) -DMQUEUE_DEPTH=$(MQUEUE_DEPTH) -DMQUEUE_PACKET_SIZE=$(MQUEUE_PACKET_SIZE) -DCOINES_BRIDGE_PROTOCOL
CFLAGS += -DSTREAM_RSP_TIMEOUT=$(STREAM_RSP_TIMEOUT) -DFIFO_STREAM_RSP_TIMEOUT=$(FIFO_STREAM_RSP_TIMEOUT)
CFLAGS += -std=c99 -D$(PLATFORM) -c -O0 -g -Wall -D$(DRIVER)

LIB_DIR=../../../libraries

C_SRCS_COINES += \
../coines_common.c \
error_handling/error_handling.c \
api/api.c \
api/board_specific/gpio/gpio.c \
api/board_specific/board_interface/board_interface.c \
api/delay/delay.c \
api/eeprom/eeprom.c \
api/stream/streaming.c \
protocol/protocol.c \
protocol/bridge/bridge_decoder.c \
protocol/bridge/bridge_encoder.c \
interface/interface.c \
platform/platform.c \
$(LIB_DIR)/mqueue_host/mqueue.c \
$(LIB_DIR)/circular_buffer/circular_buffer.c \

ifeq ($(PLATFORM), PLATFORM_WINDOWS)
	C_SRCS_COINES += \
 	platform/Windows/ble/simpleble-0.6.0/simpleble_lib_loader.c \
	platform/Windows/ble/simpleble-0.6.0/ble_windows.c \
	platform/Windows/serial/sync_comm/sync_comm_windows.c
else
	C_SRCS_COINES += \
	platform/common/ble/simpleble-0.6.0/ble_unix.c \
	platform/common/serial/sync_comm/sync_comm_unix.c
endif

C_SRCS_COINES += \
platform/common/ble/simpleble-0.6.0/ble.c


INCLUDEPATHS_COINES += \
.. \
$(LIB_DIR)/mqueue_host \
$(LIB_DIR)/circular_buffer \
error_handling \
api \
api/board_specific/gpio \
api/board_specific/board_interface \
api/delay \
api/eeprom \
api/stream \
protocol \
protocol/bridge \
interface \
platform \
platform/common/ble/simpleble-0.6.0 \
platform/common/ble/simpleble-0.6.0/includes/simpleble \
platform/common/ble/simpleble-0.6.0/includes/simpleble_c

ifeq ($(PLATFORM), PLATFORM_WINDOWS)
	INCLUDEPATHS_COINES += \
	platform/Windows/ble/simpleble-0.6.0 \
	platform/Windows/serial/sync_comm
else
	INCLUDEPATHS_COINES += \
	platform/common/serial/libusbp-1.0/includes \
	platform/common/serial/sync_comm
endif

INCLUDEPATHS_COINES += \
platform/common/ble/simpleble-0.6.0