COINES_INSTALL_PATH ?= ../../

EXAMPLE_FILE ?= app20-flash.c

override TARGET=PC
override COINES_BACKEND=COINES_LEGACY

INCLUDEPATHS += \
$(COINES_INSTALL_PATH)/coines-api/pc \
$(COINES_INSTALL_PATH)/coines-api/pc/comm_driver \
$(COINES_INSTALL_PATH)/coines-api/pc/comm_intf \

ifeq ($(OS),Windows_NT)
C_OBJS += app20-flash_rc.o
endif

include $(COINES_INSTALL_PATH)/coines.mk

ifeq ($(OS),Windows_NT)
app20-flash_rc.o : app20-flash.rc
	@echo [ RES ] $<
	@windres -i $< -o $@
endif