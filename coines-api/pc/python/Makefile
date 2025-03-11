COMPILER_PREFIX ?=
CC = $(COMPILER_PREFIX)gcc
LD = $(COMPILER_PREFIX)g++
TARGET = PC
override COINES_BACKEND = COINES_PC
COINES_INSTALL_PATH ?= ../../..
COINES_PC_PATH = ../../coines-api/pc/coines_pc

ifeq ($(COINES_BACKEND),COINES_PC)
    #--- Configure size based on the application need --#
    CBUFF_SIZE ?= 104857600 # 100MB
    MQUEUE_DEPTH ?= 100
    #-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # FIFO streaming is supported in interrupt streaming mode only. So packet size need to be configured based on the application need.
    # Example : MQUEUE_PACKET_SIZE ?= 255 Interrupt streaming
    # Example : MQUEUE_PACKET_SIZE ?= 548 (Watermark level(framelength * samples) + packet counter(4bytes) + dummy bytes(2bytes) + Ignore bytes if any may be frame header(2bytes))
    #                                      Watermark level(18 * 30) + 4 + 2 + 2= 548    
    #-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------#                
    MQUEUE_PACKET_SIZE ?= 255 # Streaming max 3KB supported.
    #---------------------------------------------------------------------#
    #----- Default value configured inside library considering lower ODR 0.78Hz -----#
    #                  STREAM_RSP_TIMEOUT_MS = 1500                                  #
    #                  FIFO_STREAM_RSP_TIMEOUT_MS = 10000                            #
    #--------------------------------------------------------------------------------#
    # Configure timeout based on application need
    STREAM_RSP_TIMEOUT ?= 0 #Millisecond
    FIFO_STREAM_RSP_TIMEOUT ?= 0 #Millisecond
    #---------------------------------------------------------------------#
    
    CFLAGS += -DCIRCULAR_BUFFER_SIZE=$(CBUFF_SIZE) -DMQUEUE_DEPTH=$(MQUEUE_DEPTH) -DMQUEUE_PACKET_SIZE=$(MQUEUE_PACKET_SIZE) -DCOINES_BRIDGE_PROTOCOL 
    CFLAGS += -DSTREAM_RSP_TIMEOUT=$(STREAM_RSP_TIMEOUT) -DFIFO_STREAM_RSP_TIMEOUT=$(FIFO_STREAM_RSP_TIMEOUT)
    
endif

# On using Software reset API(coines_soft_reset()) from COINES, After reset device jumps to the address specified in APP_START_ADDRESS.
export APP_START_ADDRESS ?= 0x00030000

ARCH = $(shell $(CC) -dumpmachine)

ifneq (,$(findstring x86_64,$(ARCH)))
    SUFFIX = 64
else
    ifneq (,$(findstring arm64,$(ARCH)))
        SUFFIX = 64
    else
        SUFFIX = 32
    endif
endif

ifeq ($(OS),Windows_NT)
    PLATFORM = PLATFORM_WINDOWS
    DRIVER = LEGACY_USB_DRIVER
    ifneq ($(findstring 32,$(SUFFIX)),)
        LIBS += pthread
        CFLAGS += -m32 -D PLATFORM_WINDOWS -std=c99 -DAPP_START_ADDRESS=$(APP_START_ADDRESS)
    else
        CFLAGS += -m64 -D PLATFORM_WINDOWS -std=c99 -DAPP_START_ADDRESS=$(APP_START_ADDRESS)
    endif
    VARIANT = win$(SUFFIX)
    EXT = dll
    LIBS += setupapi
    ifeq ($(notdir $(MAKE)),mingw32-make)
        SHELL = cmd
        RM  = del /s /q
        MKDIR = mkdir
        syspath = $(subst /,\,$(1))
    else
        RM = rm -rf
        MKDIR = mkdir -p
        syspath = $(subst /,/,$(1))
    endif
else
    PLATFORM = unix
    DRIVER = LIBUSB_DRIVER
    ifneq ($(findstring 32,$(SUFFIX)),)
        CFLAGS += -D PLATFORM_LINUX -fPIC -std=c17 -m32 -DAPP_START_ADDRESS=$(APP_START_ADDRESS)
    else
        CFLAGS += -D PLATFORM_LINUX -fPIC -std=c17 -m64 -DAPP_START_ADDRESS=$(APP_START_ADDRESS)
    endif

    ifeq ($(shell uname -s),Linux)
        EXT = so
        VARIANT = linux$(SUFFIX)
    else ifeq ($(shell uname -s),Darwin)
        EXT = dylib
        VARIANT = mac$(SUFFIX)
    endif

    ifeq ($(COINES_BACKEND), $(filter $(COINES_BACKEND), COINES_BRIDGE COINES_PC))
        ifeq ($(shell uname -s),Linux)
            HOST_LIB_PATH = Linux
            HOST_OS = Linux
        else ifeq ($(shell uname -s),Darwin)
            HOST_LIB_PATH = Mac
            HOST_OS = Darwin
        endif
        ifeq ($(shell uname -m),arm64)
            HOST_ARCH = arm64
            LIBPATHS += /opt/homebrew/lib 
        else ifneq ($(findstring 64,$(SUFFIX)),)
            HOST_ARCH = x64
        else ifneq ($(findstring 32,$(SUFFIX)),)
            HOST_ARCH = x86
        endif

        DRIVER = LIBUSBP_DRIVER

		ifeq ($(COINES_BACKEND), COINES_PC)
			LIB_PATH_ARCH = $(COINES_INSTALL_PATH)/coines-api/pc/coines_pc/platform/$(HOST_LIB_PATH)/serial/libusbp-1.0/libs/$(HOST_ARCH)
        	BLE_LIB_PATH_ARCH = $(COINES_INSTALL_PATH)/coines-api/pc/coines_pc/platform/$(HOST_LIB_PATH)/ble/simpleble-0.6.0/libs/$(HOST_ARCH)
		else
			LIB_PATH = $(COINES_INSTALL_PATH)/coines-api/pc/serial_com/libusbp-1.0
			LIB_PATH_ARCH += $(LIB_PATH)/$(HOST_OS)/$(HOST_ARCH)
			BLE_LIB_PATH = $(COINES_INSTALL_PATH)/coines-api/pc/ble_com/simpleble-0.6.0
			BLE_LIB_PATH_ARCH += $(BLE_LIB_PATH)/$(HOST_OS)/$(HOST_ARCH)
		endif
    else
        DRIVER = LIBUSB_DRIVER
        INCLUDEPATHS_COINES += comm_driver/libusb-1.0
        ifeq ($(shell uname -m),arm64)
            LIBPATHS += /opt/homebrew/lib   
        endif
    endif

    ifeq ($(DRIVER),LIBUSB_DRIVER)
        LIBS += usb-1.0
    endif

    ifeq ($(DRIVER),LIBUSBP_DRIVER)
        ifeq ($(HOST_OS),Linux)
            EXTRA_LIBS += -l:libusbp-1.a
            EXTRA_LIBS += -ludev
            EXTRA_LIBS += -l:libsimpleble-c.a
            EXTRA_LIBS += -l:libsimpleble.a
            EXTRA_LIBS += -ldbus-1
            EXTRA_LIBS += -lpthread 
        endif
        ifeq ($(shell uname -s),Darwin)
            EXTRA_LIBS += -framework IOKit -framework CoreFoundation -framework Foundation -framework CoreBluetooth
            EXTRA_LIBS += $(LIB_PATH_ARCH)/libusbp-1.a
            EXTRA_LIBS += $(BLE_LIB_PATH_ARCH)/libsimpleble-c.a
            EXTRA_LIBS += $(BLE_LIB_PATH_ARCH)/libsimpleble.a
        endif
    endif

    MKDIR = mkdir -p
    RM  = rm -rf
    syspath = $(subst /,/,$(1))

endif

LIBPATHS += \
$(LIB_PATH_ARCH) \
$(BLE_LIB_PATH_ARCH) \
$(COINES_INSTALL_PATH)/coines-api \

OBJ_DIR = build/$(VARIANT)

SRC_DIR = ..
ifeq ($(COINES_BACKEND),COINES_PC)
	C_SRCS_COINES += \
	../coines_common.c \
	$(COINES_PC_PATH)/error_handling/error_handling.c \
	$(COINES_PC_PATH)/api/api.c \
	$(COINES_PC_PATH)/api/board_specific/gpio/gpio.c \
	$(COINES_PC_PATH)/api/board_specific/board_interface/board_interface.c \
	$(COINES_PC_PATH)/api/delay/delay.c \
	$(COINES_PC_PATH)/api/eeprom/eeprom.c \
	$(COINES_PC_PATH)/api/stream/streaming.c \
	$(COINES_PC_PATH)/protocol/protocol.c \
	$(COINES_PC_PATH)/protocol/bridge/bridge_decoder.c \
	$(COINES_PC_PATH)/protocol/bridge/bridge_encoder.c \
	$(COINES_PC_PATH)/interface/interface.c \
	$(COINES_PC_PATH)/platform/platform.c \
    ../../libraries/mqueue_host/mqueue.c \
    ../../libraries/circular_buffer/circular_buffer.c


	ifeq ($(PLATFORM), PLATFORM_WINDOWS)
		C_SRCS_COINES += \
		$(COINES_PC_PATH)/platform/Windows/ble/simpleble-0.6.0/simpleble_lib_loader.c \
		$(COINES_PC_PATH)/platform/Windows/ble/simpleble-0.6.0/ble_windows.c \
		$(COINES_PC_PATH)/platform/Windows/serial/sync_comm/sync_comm_windows.c
	else
		C_SRCS_COINES += \
		$(COINES_PC_PATH)/platform/common/ble/simpleble-0.6.0/ble_unix.c \
		$(COINES_PC_PATH)/platform/common/serial/sync_comm/sync_comm_unix.c
	endif

	C_SRCS_COINES += \
	$(COINES_PC_PATH)/platform/common/ble/simpleble-0.6.0/ble.c


	INCLUDEPATHS_COINES += \
	.. \
    ../../libraries/mqueue_host \
    ../../libraries/circular_buffer \
	$(COINES_PC_PATH)/error_handling \
	$(COINES_PC_PATH)/api \
	$(COINES_PC_PATH)/api/board_specific/gpio \
	$(COINES_PC_PATH)/api/board_specific/board_interface \
	$(COINES_PC_PATH)/api/delay \
	$(COINES_PC_PATH)/api/eeprom \
	$(COINES_PC_PATH)/api/stream \
	$(COINES_PC_PATH)/protocol \
	$(COINES_PC_PATH)/protocol/bridge \
	$(COINES_PC_PATH)/interface \
	$(COINES_PC_PATH)/platform \
	$(COINES_PC_PATH)/platform/common/ble/simpleble-0.6.0 \
	$(COINES_PC_PATH)/platform/common/ble/simpleble-0.6.0/includes/simpleble \
	$(COINES_PC_PATH)/platform/common/ble/simpleble-0.6.0/includes/simpleble_c 


	ifeq ($(PLATFORM), PLATFORM_WINDOWS)
		INCLUDEPATHS_COINES += \
		$(COINES_PC_PATH)/platform/Windows/ble/simpleble-0.6.0 \
		$(COINES_PC_PATH)/platform/Windows/serial/sync_comm
	else
		INCLUDEPATHS_COINES += \
		$(COINES_PC_PATH)/platform/common/serial/libusbp-1.0/includes \
		$(COINES_PC_PATH)/platform/common/serial/sync_comm
	endif

	INCLUDEPATHS_COINES += \
	$(COINES_PC_PATH)/platform/common/ble/simpleble-0.6.0

else ifeq ($(COINES_BACKEND),COINES_BRIDGE)
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
    .. \
    serial_com \
    ble_com \
    ble_com/simpleble-0.6.0/simpleble \
    ble_com/simpleble-0.6.0/simpleble_c

else
    CFLAGS += -D $(DRIVER)

    C_SRCS_COINES += \
    ../coines_common.c \
    coines.c \
    comm_intf/comm_intf.c \
    comm_intf/comm_ringbuffer.c \
    comm_driver/usb.c

    ifeq ($(PLATFORM), PLATFORM_WINDOWS)
        C_SRCS_COINES += \
        comm_driver/legacy_usb/legacy_usb_support.c
    endif

    INCLUDEPATHS_COINES += \
    . \
    .. \
    coines_api \
    comm_intf \
    comm_driver \
    comm_driver/legacy_usb

endif

C_SRCS = $(addprefix $(SRC_DIR)/,$(C_SRCS_COINES))
INCLUDEPATHS = $(addprefix $(SRC_DIR)/,$(INCLUDEPATHS_COINES))

C_FILES = $(notdir $(C_SRCS))
C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.c.o))
C_PATHS = $(sort $(dir $(C_SRCS)))
DEP = $(C_OBJS:%.o=%.d)

vpath %.c $(C_PATHS)

CFLAGS += -c -D $(TARGET)
#CFLAGS += -lstdc++

# Define COINES_VERSION as Git revision string, if
# - Git is installed
# - it was run inside a Git repository
# ifneq ($(shell $(WHERE) git) ,)
# GIT_REVISION_STRING = $(shell git describe  --always --dirty --tags)
# ifneq ($(GIT_REVISION_STRING),)
# CFLAGS += -D COINES_VERSION=\"$(GIT_REVISION_STRING)\"
# endif
# endif

LDFLAGS += -shared
ifneq ($(findstring 32,$(SUFFIX)),)
    LDFLAGS += -m32
endif

ifeq ($(HOST_OS),Darwin)
    ifeq ($(HOST_ARCH),arm64)
        SHARED_LIB += coinespy/libcoines_arm_$(SUFFIX).$(EXT)
    else
        SHARED_LIB += coinespy/libcoines_i386_$(SUFFIX).$(EXT)
    endif    
else
    SHARED_LIB += coinespy/libcoines_$(SUFFIX).$(EXT)
endif

all: $(SHARED_LIB)
	@echo CFLAGS_SAVE = $(CFLAGS) > $(OBJ_DIR)/cflags.save

$(OBJ_DIR):
	@echo [ MKDIR ] $@
	@$(MKDIR) $(call syspath,$@)

$(SHARED_LIB): $(OBJ_DIR) $(C_OBJS)
	@echo [ LD ] $@
	@$(LD) $(LDFLAGS) -o "$@" $(C_OBJS) $(addprefix -L,$(LIBPATHS)) $(addprefix -l,$(LIBS)) $(EXTRA_LIBS)

-include $(DEP)

$(OBJ_DIR)/%.c.o: %.c
	@echo [ CC ] $<
	@$(CC) $(CFLAGS) -MMD -I . $(addprefix -I,$(INCLUDEPATHS)) -o "$@" "$<"

clean:
	@echo "Cleaning..."
	@$(RM) $(call syspath,$(SHARED_LIB)) build