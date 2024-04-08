ifeq ($(OS),Windows_NT)
    PLATFORM = PLATFORM_WINDOWS
    DRIVER ?= LEGACY_USB_DRIVER
else
    PLATFORM = PLATFORM_LINUX
    DRIVER = LIBUSB_DRIVER
endif

CC     = gcc

CFLAGS += -std=c99 -D$(PLATFORM) -c -O0 -g -Wall -D$(DRIVER)

ifeq ($(COINES_BACKEND),COINES_BRIDGE)
    C_SRCS_COINES += \
    ../coines_common.c \
    coines_bridge.c \
    serial_com/serial_com.c \
    ble_com/ble_com.c 

    ifeq ($(PLATFORM), PLATFORM_WINDOWS)
        C_SRCS_COINES += ble_com/simpleble_lib_loader.c
    endif

    INCLUDEPATHS_COINES += \
    . \
    serial_com \
    ble_com \
    ble_com/simpleble-0.6.0/simpleble \
    ble_com/simpleble-0.6.0/simpleble_c \

else
    C_SRCS_COINES += \
    ../coines_common.c \
    coines.c \
    comm_intf/comm_intf.c \
    comm_intf/comm_ringbuffer.c \
    comm_driver/usb.c \

    INCLUDEPATHS_COINES += \
    . \
    coines_api \
    comm_intf \
    comm_driver \

    ifeq ($(DRIVER),LEGACY_USB_DRIVER)
		C_SRCS_COINES += comm_driver/legacy_usb/legacy_usb_support.c
		INCLUDEPATHS_COINES += comm_driver/legacy_usb
	endif

	ifeq ($(DRIVER),LIBUSB_DRIVER)
		INCLUDEPATHS_COINES += comm_driver/libusb-1.0
	endif

endif



