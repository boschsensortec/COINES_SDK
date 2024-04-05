ifeq ($(OS),Windows_NT)
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
    WHERE = where
else
    MKDIR = mkdir -p
    RM  = rm -rf
    syspath = $(subst /,/,$(1))
    WHERE = which
endif

TARGET ?= PC
# On using Software reset API(coines_soft_reset()) from COINES, After reset device jumps to the address specified in APP_START_ADDRESS.
APP_START_ADDRESS ?=0x00030000

ifeq ($(TARGET),PC)
SRC_DIR = pc
include $(SRC_DIR)/pc.mk
STATIC_LIB = libcoines-pc.a
endif

ifeq ($(TARGET),MCU_APP20)
SRC_DIR = mcu_app20
include $(SRC_DIR)/mcu_app20.mk
STATIC_LIB = libcoines-mcu_app20.a
endif

ifeq ($(TARGET),MCU_APP30)
SRC_DIR = mcu_app30
include $(SRC_DIR)/mcu_app30.mk
STATIC_LIB = libcoines-mcu_app30.a
endif

ifeq ($(TARGET),MCU_NICLA)
SRC_DIR = mcu_nicla
include $(SRC_DIR)/mcu_nicla.mk
STATIC_LIB = libcoines-mcu_nicla.a
endif

ifeq ($(TARGET),MCU_APP31)
SRC_DIR = mcu_app31
include $(SRC_DIR)/mcu_app31.mk
STATIC_LIB = libcoines-mcu_app31.a
endif

OBJ_DIR = build/$(TARGET)

ASM_SRCS = $(addprefix $(SRC_DIR)/,$(ASM_SRCS_COINES))
C_SRCS = $(addprefix $(SRC_DIR)/,$(C_SRCS_COINES))
INCLUDEPATHS = $(addprefix $(SRC_DIR)/,$(INCLUDEPATHS_COINES))

ASM_FILES = $(notdir $(ASM_SRCS))
ASM_OBJS = $(addprefix $(OBJ_DIR)/, $(ASM_FILES:.S=.S.o))
ASM_PATHS = $(sort $(dir $(ASM_SRCS)))

C_FILES = $(notdir $(C_SRCS))
C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.c.o))
C_PATHS = $(sort $(dir $(C_SRCS)))
DEP = $(C_OBJS:%.o=%.d)

vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

CFLAGS += -D $(TARGET)  -DAPP_START_ADDRESS=$(APP_START_ADDRESS)

GIT_REV_STR_ENABLED ?= 0
# Define COINES_VERSION as Git revision string, if
# - Git is installed 
# - it was run inside a Git repository
ifneq ($(GIT_REV_STR_ENABLED), 0)
ifneq ($(shell $(WHERE) git) ,)
GIT_REV_STR = $(shell git describe  --always --dirty --tags)
ifneq ($(GIT_REV_STR),)
CFLAGS += -D COINES_VERSION=\"$(GIT_REV_STR)\"
endif
endif
endif

ARFLAGS = -rcs

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
all: $(STATIC_LIB)
	@echo CFLAGS_SAVE = $(CFLAGS) > $(OBJ_DIR)/cflags.save

$(OBJ_DIR):
	@echo [ MKDIR ] $@
	@$(MKDIR) $(call syspath,$@)
	
$(STATIC_LIB): $(OBJ_DIR) $(C_OBJS) $(ASM_OBJS)
	@echo [ AR ] $@
	@$(AR) $(ARFLAGS) "$@" $(C_OBJS) $(ASM_OBJS)

-include $(DEP)

$(OBJ_DIR)/%.S.o: %.S
	@echo [ AS ] $<
	@$(CC) $(CFLAGS) -o "$@" "$<"

$(OBJ_DIR)/%.c.o: %.c
	@echo [ CC ] $< 
	@$(CC) $(CFLAGS) -MMD -I . $(addprefix -I,$(INCLUDEPATHS)) -o "$@" "$<"

clean:
	@echo "Cleaning..."
	@$(RM) $(STATIC_LIB) build
	
clean_pc:
ifneq ("$(wildcard $(STATIC_LIB))","")
	@echo "Cleaning pc..."
	@$(RM) $(STATIC_LIB) $(call syspath,$(OBJ_DIR))
endif

.PHONY: all clean $(STATIC_LIB)

