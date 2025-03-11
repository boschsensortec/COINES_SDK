TARGET ?= PC
USE_FREERTOS?=0

# COMPort name to download the binary
COM_PORT ?=

# On using Software reset API(coines_soft_reset()) from COINES_SDK , After reset device jumps to the address specified in APP_START_ADDRESS.
APP_START_ADDRESS ?=0x00030000

OBJ_DIR = build/$(TARGET)

ifneq ($(TARGET),$(filter $(TARGET),PC MCU_APP30 MCU_NICLA MCU_APP31 MCU_HEAR3X))
    $(info Unsupported 'TARGET' : $(TARGET))
    $(info Supported 'TARGET's : PC, MCU_APP30, MCU_NICLA, MCU_APP31 , MCU_HEAR3X)
    $(error Exit)
endif

COINES_BACKEND ?= COINES_PC
ifeq ($(COINES_BACKEND),COINES_BRIDGE)
    override COINES_BACKEND = COINES_PC
endif

ifneq ($(LOCATION),$(filter $(LOCATION),RAM FLASH))
    $(info Unsupported 'LOCATION' : $(LOCATION))
    $(info Supported 'LOCATION's : RAM, FLASH)
    $(error Exit)
endif

# Compiler optimization level
OPT ?= -Os

# Debug flags, Set to 0 (disable) or 1 (enable)
DEBUG ?= 0
ifeq ($(DEBUG),0)
CFLAGS += -U DEBUG -D NDEBUG
else
CFLAGS += -D DEBUG -U NDEBUG
endif

# Pre-charge enable for APP board 3.1
PRE_CHARGE_EN ?= 0

################################ MCU Target common - APP3.0,NICLA,MCU_APP31 ############################
ifeq ($(TARGET),$(filter $(TARGET),MCU_APP30 MCU_NICLA MCU_APP31 MCU_HEAR3X))
    CFLAGS += -std=c99 -mthumb -mabi=aapcs -mcpu=cortex-m4 -c $(OPT) -g3 -Wall -D$(TARGET) -DAPP_START_ADDRESS=$(APP_START_ADDRESS) -ffunction-sections -fdata-sections
    CPPFLAGS += -mthumb -mabi=aapcs -mcpu=cortex-m4 -c $(OPT) -g3 -Wall -D$(TARGET) -ffunction-sections -fdata-sections

    LDFLAGS += -mthumb -mcpu=cortex-m4 -specs=nano.specs -Wl,--cref -Wl,--check-sections \
               -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all \
               -Xlinker -Map=$(OBJ_DIR)/build.map -Wl,--start-group -u _printf_float -u _exit -Wl,--end-group

    CROSS_COMPILE = arm-none-eabi-
    # LOCATION ?= FLASH
    LOCATION ?= RAM
    EXT = .elf
endif
#############################################################################################

ifeq ($(TARGET),PC)
ifeq ($(COINES_BACKEND), COINES_LEGACY)
CFLAGS += -DCOINES_LEGACY
else
CFLAGS += -DCOINES_BRIDGE
endif
CFLAGS += -std=gnu99 -c -g3 $(OPT) -D$(TARGET) -Wall
CPPFLAGS += -c -g3 $(OPT) -D$(TARGET) -Wall
CROSS_COMPILE =
endif

AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AR = $(CROSS_COMPILE)ar
OBJCOPY = $(CROSS_COMPILE)objcopy
APP_SWITCH_PATH = $(COINES_INSTALL_PATH)/tools/app_switch

ifeq ($(OS),Windows_NT)
    IS_CC_FOUND = $(shell where $(CC))
    # Check whether app_switch executable is available or not
    ifneq ($(findstring app_switch.exe, $(wildcard $(APP_SWITCH_PATH)/*)),)
        IS_APP_SWITCH_FOUND = $(APP_SWITCH_PATH)
    else
        IS_APP_SWITCH_FOUND =
    endif

    $(info Platform: Windows)
    PLATFORM = PLATFORM_WINDOWS
    LIB_PATH ?= $(COINES_INSTALL_PATH)/coines-api/pc/comm_driver/libusb-1.0/mingw_lib
    DRIVER ?= LEGACY_USB_DRIVER
    ifneq ($(IS_CC_FOUND),)
      GCC_TARGET = $(shell gcc -dumpmachine)
    endif

    ifneq (,$(findstring x86_64,$(GCC_TARGET)))
        LIB_PATH_ARCH ?= $(LIB_PATH)/x64
    else
        LIB_PATH_ARCH ?= $(LIB_PATH)/x86
        EXTRA_LIBS += -lpatch
    endif
    ifeq ($(notdir $(MAKE)),mingw32-make)
        SHELL = cmd
        CP  = copy
        RM  = del /s /q
        MKDIR = mkdir
        syspath = $(subst /,\,$(1))
    else
        CP = cp
        RM = rm -rf
        MKDIR = mkdir -p
        syspath = $(subst /,/,$(1))
    endif
    DFU = $(COINES_INSTALL_PATH)/tools/usb-dfu/dfu-util
else
    IS_CC_FOUND = $(shell which $(CC))
    IS_APP_SWITCH_FOUND = $(shell which $(APP_SWITCH))
    $(info Platform: Linux / macOS)
    PLATFORM = PLATFORM_LINUX

    ifeq ($(COINES_BACKEND), $(filter $(COINES_BACKEND), COINES_BRIDGE COINES_PC))
        ifeq ($(shell uname -s),Linux)
            HOST_OS = Linux
        else ifeq ($(shell uname -s),Darwin)
			ifeq ($(COINES_BACKEND), COINES_PC)
				HOST_OS = Mac
			else
				HOST_OS = Darwin
			endif
        endif
        ifeq ($(shell uname -m),x86_64)
            HOST_ARCH = x64
        else ifneq ($(filter %86 ,$(shell uname -m)),)
            HOST_ARCH = x86
        else ifeq ($(shell uname -m),arm64)
            HOST_ARCH = arm64
            LIBPATHS += /opt/homebrew/lib
        endif

        DRIVER = LIBUSBP_DRIVER

		ifeq ($(COINES_BACKEND), COINES_PC)
			LIB_PATH_ARCH = $(COINES_INSTALL_PATH)/coines-api/pc/coines_pc/platform/$(HOST_OS)/serial/libusbp-1.0/libs/$(HOST_ARCH)
        	BLE_LIB_PATH_ARCH = $(COINES_INSTALL_PATH)/coines-api/pc/coines_pc/platform/$(HOST_OS)/ble/simpleble-0.6.0/libs/$(HOST_ARCH)
		else
			LIB_PATH = $(COINES_INSTALL_PATH)/coines-api/pc/serial_com/libusbp-1.0
			LIB_PATH_ARCH += $(LIB_PATH)/$(HOST_OS)/$(HOST_ARCH)
			BLE_LIB_PATH = $(COINES_INSTALL_PATH)/coines-api/pc/ble_com/simpleble-0.6.0
			BLE_LIB_PATH_ARCH += $(BLE_LIB_PATH)/$(HOST_OS)/$(HOST_ARCH)
		endif
    else
        DRIVER = LIBUSB_DRIVER
        ifeq ($(shell uname -m),arm64)
            LIBPATHS += /opt/homebrew/lib
        endif
    endif
    RM = rm -rf
    DFU = dfu-util
    MKDIR = mkdir -p
    syspath = $(subst /,/,$(1))
endif

APP_SWITCH = $(COINES_INSTALL_PATH)/tools/app_switch/app_switch
OPEN_OCD = $(COINES_INSTALL_PATH)/tools/openocd/xpack-openocd-0.11.0-4/bin/openocd

#################################  Common - PC and MCU  #####################################

ifeq ($(IS_CC_FOUND),)
    $(error cc: $(CC) not found / Add $(CC) Folder to PATH)
else
    $(info cc:  "$(IS_CC_FOUND)".)
endif


ifeq ($(suffix $(EXAMPLE_FILE)),.S)
ASM_SRCS += $(EXAMPLE_FILE)
endif

ifeq ($(suffix $(EXAMPLE_FILE)),.c)
C_SRCS += $(EXAMPLE_FILE)
endif

ifeq ($(suffix $(EXAMPLE_FILE)),.cpp)
CPP_SRCS += $(EXAMPLE_FILE)
endif

PROJ_NAME = $(basename $(EXAMPLE_FILE))
EXE ?=  $(PROJ_NAME)$(EXT)
BIN ?=  $(PROJ_NAME).bin
HEX ?=  $(PROJ_NAME).hex
APPSWITCH =appswitch

INCLUDEPATHS += $(COINES_INSTALL_PATH)/coines-api

LIBPATHS += \
$(LIB_PATH_ARCH) \
$(BLE_LIB_PATH_ARCH) \
$(COINES_INSTALL_PATH)/coines-api \

################################ MCU Target - APP3.0 specific #################################

ifeq ($(TARGET),MCU_APP30)
    DEVICE_ID = -,108c:ab3d
        ifeq ($(LOCATION),RAM)
            LD_SCRIPT = $(COINES_INSTALL_PATH)/coines-api/mcu_app30/linker_scripts/mcu_app30_ram_ble.ld
        else
            LD_SCRIPT = $(COINES_INSTALL_PATH)/coines-api/mcu_app30/linker_scripts/mcu_app30_flash_ble.ld
        endif
    CFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    CPPFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T $(LD_SCRIPT)

    # Doesn't work for some reason !
    # LIBS += coines-mcu_app30
    LDFLAGS += -Wl,--whole-archive -L $(COINES_INSTALL_PATH) -lcoines-mcu_app30 -Wl,--no-whole-archive
    ARTIFACTS = $(EXE) $(BIN) $(HEX)

endif

################################ MCU Target - NICLA specific #################################

ifeq ($(TARGET),MCU_NICLA)

    LD_SCRIPT = $(COINES_INSTALL_PATH)/coines-api/mcu_nicla/linker_scripts/mcu_nicla_flash_ble.ld
    CFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    CPPFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T $(LD_SCRIPT)

    LDFLAGS += -Wl,--whole-archive -L $(COINES_INSTALL_PATH) -lcoines-mcu_nicla -Wl,--no-whole-archive
    ARTIFACTS = $(EXE) $(BIN) $(HEX)

endif

################################ MCU Target - APP3.1 specific #################################

ifeq ($(TARGET),MCU_APP31)
    DEVICE_ID = -,108c:ab39
        ifeq ($(LOCATION),RAM)
            LD_SCRIPT = $(COINES_INSTALL_PATH)/coines-api/mcu_app31/linker_scripts/mcu_app31_ram_ble.ld
        else
            LD_SCRIPT = $(COINES_INSTALL_PATH)/coines-api/mcu_app31/linker_scripts/mcu_app31_flash_ble.ld
        endif
    CFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    CPPFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T $(LD_SCRIPT)

    # Doesn't work for some reason !
    # LIBS += coines-mcu_app31
    LDFLAGS += -Wl,--whole-archive -L $(COINES_INSTALL_PATH) -lcoines-mcu_app31 -Wl,--no-whole-archive
    ARTIFACTS = $(EXE) $(BIN) $(HEX)

endif
################################ MCU Target - Hearable specific ############################
ifeq ($(TARGET),MCU_HEAR3X)
    DEVICE_ID = -,108c:4b3d
        ifeq ($(LOCATION),RAM)
            LD_SCRIPT = $(COINES_INSTALL_PATH)/coines-api/mcu_hear3x/linker_scripts/mcu_hear3x_ram_ble.ld
        else
            LD_SCRIPT = $(COINES_INSTALL_PATH)/coines-api/mcu_hear3x/linker_scripts/mcu_hear3x_flash_ble.ld
        endif
    CFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    CPPFLAGS +=  -mfloat-abi=hard -mfpu=fpv4-sp-d16
    LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T $(LD_SCRIPT)

    # Doesn't work for some reason !
    # LIBS += coines-mcu_hear3x
    LDFLAGS += -Wl,--whole-archive -L $(COINES_INSTALL_PATH) -lcoines-mcu_hear3x -Wl,--no-whole-archive
    ARTIFACTS = $(EXE) $(BIN) $(HEX)

endif

################################ PC Target - Windows,Linux/macOS ############################

ifeq ($(TARGET),PC)
    CFLAGS += -D$(PLATFORM)
    CPPFLAGS += -D$(PLATFORM)
	LIBS += coines-pc

    ifeq ($(PLATFORM),PLATFORM_LINUX)
        LIBS += pthread
    endif

    ifeq ($(PLATFORM),PLATFORM_WINDOWS)
        ifeq (,$(findstring x86_64,$(GCC_TARGET)))
            LIBS += pthread
        endif
    endif

    ifeq ($(DRIVER),LEGACY_USB_DRIVER)
        LIBS += setupapi
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
        endif
        ifeq ($(shell uname -s),Darwin)
            EXTRA_LIBS += -framework IOKit -framework CoreFoundation -framework Foundation -framework CoreBluetooth
            EXTRA_LIBS += $(LIB_PATH_ARCH)/libusbp-1.a
            EXTRA_LIBS += $(BLE_LIB_PATH_ARCH)/libsimpleble-c.a
            EXTRA_LIBS += $(BLE_LIB_PATH_ARCH)/libsimpleble.a
        endif
    endif

    ARTIFACTS = $(EXE)
endif

#############################################################################################

LIBS += m

ASM_FILES = $(notdir $(ASM_SRCS))
ASM_OBJS += $(addprefix $(OBJ_DIR)/, $(ASM_FILES:.S=.S.o))
ASM_PATHS = $(sort $(dir $(ASM_SRCS)))
vpath %.S $(ASM_PATHS)

C_FILES = $(notdir $(C_SRCS))
C_OBJS += $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.c.o))
C_PATHS = $(sort $(dir $(C_SRCS)))
DEP += $(C_OBJS:%.o=%.d)
vpath %.c $(C_PATHS)

CPP_FILES = $(notdir $(CPP_SRCS))
CPP_OBJS += $(addprefix $(OBJ_DIR)/, $(CPP_FILES:.cpp=.cpp.o))
CPP_PATHS = $(sort $(dir $(CPP_SRCS)))
DEP += $(CPP_OBJS:%.o=%.d)
vpath %.cpp $(CPP_PATHS)

#Load contents of cflags.save file.
#Populates the the CFLAGS_SAVE variable
-include $(OBJ_DIR)/cflags.save


#Compare  CFLAGS_SAVE with CFLAGS,if they differ perform a clean build
#If CFLAGS_SAVE is empty, don't do anything
ifneq ($(CFLAGS_SAVE),)
ifneq ($(strip $(CFLAGS)),$(strip $(CFLAGS_SAVE)))
ifneq (,$(shell $(RM) $(call syspath,$(OBJ_DIR))))
$(info Cleaning...)
endif
endif
endif

####################################################################
# Make Targets                                                     #
####################################################################
all: $(ARTIFACTS)
	@echo CFLAGS_SAVE = $(CFLAGS) > $(OBJ_DIR)/cflags.save
	$(call syspath,$(POSTBUILD_CMD))

$(OBJ_DIR):
	@echo [ MKDIR ] $@
	@$(MKDIR) $(call syspath,$@)

$(BIN): $(EXE)
	@echo [ BIN ] $@
	@$(OBJCOPY) -O binary $< $@

$(HEX): $(EXE)
	@echo [ HEX ] $@
	@$(OBJCOPY) -O ihex $< $@

$(EXE): $(OBJ_DIR) $(C_OBJS) $(CPP_OBJS) $(ASM_OBJS)
ifeq ($(TARGET),PC)
	@$(MAKE) -s -C  $(COINES_INSTALL_PATH)/coines-api clean_pc
endif
	@echo [ MAKE ] coines-api
	@$(MAKE) -s -C  $(COINES_INSTALL_PATH)/coines-api TARGET=$(TARGET) OPT=$(OPT) DEBUG=$(DEBUG) COINES_BACKEND=$(COINES_BACKEND) PRE_CHARGE_EN=$(PRE_CHARGE_EN) USE_FREERTOS=$(USE_FREERTOS) USE_RTC_CLOCK=$(USE_RTC_CLOCK) FREERTOS_CONFIG_PATH=$(FREERTOS_CONFIG_PATH)
	@echo [ LD ] $@
	@$(CXX) $(LDFLAGS) -o "$@" $(C_OBJS) $(CPP_OBJS) $(ASM_OBJS) $(addprefix -L,$(LIBPATHS)) $(addprefix -l,$(LIBS)) $(EXTRA_LIBS)

-include $(DEP)

$(OBJ_DIR)/%.S.o: %.S
	@echo [ AS ] $<
	@$(CC) $(CFLAGS) -o "$@" "$<"

$(OBJ_DIR)/%.c.o: %.c
	@echo [ CC ] $<
	@$(CC) $(CFLAGS) -MMD $(addprefix -I,$(INCLUDEPATHS)) -o "$@" "$<"

$(OBJ_DIR)/%.cpp.o: %.cpp
	@echo [ CXX ] $<
	@$(CXX) $(CPPFLAGS) -MMD $(addprefix -I,$(INCLUDEPATHS)) -o "$@" "$<"

ifeq ($(IS_APP_SWITCH_FOUND),)
$(APPSWITCH):
	@echo "Building app_switch"
	@$(MAKE) -s -C  $(COINES_INSTALL_PATH)/tools/app_switch clean-all
	@$(MAKE) -s -C  $(COINES_INSTALL_PATH)/tools/app_switch
endif

ifeq ($(TARGET),$(filter $(TARGET),MCU_APP30 MCU_APP31 MCU_HEAR3X))
download: $(APPSWITCH) $(BIN)
	@$(APP_SWITCH) usb_dfu_bl $(COM_PORT)
	@echo [ DFU ] $<
	@$(DFU) --device $(DEVICE_ID) -a $(LOCATION) -D $(BIN) -R
endif
ifeq ($(TARGET),$(filter $(TARGET),MCU_NICLA))
download: $(HEX)
	@"$(OPEN_OCD)" -d2 -s "$(LIB_PATH)" -f interface/cmsis-dap.cfg -c "transport select swd; adapter speed 1000" -f target/nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter speed 10000;" -c "program $(HEX)" -c "reset run; shutdown"
endif

run:
	@$(APP_SWITCH) example

clean:
ifneq ($(wildcard $(OBJ_DIR)), )
	@echo "Cleaning..."
	@$(RM) $(ARTIFACTS) $(call syspath,$(OBJ_DIR))
endif

clean-all: clean
ifneq ($(wildcard $(OBJ_DIR)), )
	@$(RM) build $(PROJ_NAME) $(PROJ_NAME).elf $(PROJ_NAME).exe $(PROJ_NAME).bin
	@$(MAKE) -s -C  $(COINES_INSTALL_PATH)/coines-api clean
endif

.PHONY: all clean clean-all download $(ARTIFACTS) appswitch