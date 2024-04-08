CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

CFLAGS += -D__SAM4S16C__  -DUDD_ENABLE  -mthumb -mcpu=cortex-m4 -c -Os -g -Wall
CFLAGS += -ffunction-sections -fdata-sections

ASF_DIR = ../../thirdparty/xdk-asf

C_SRCS_COINES += \
../coines_common.c \
mcu_app20.c \
$(ASF_DIR)/sam/utils/cmsis/sam4s/source/templates/system_sam4s.c \
$(ASF_DIR)/sam/utils/cmsis/sam4s/source/templates/gcc/startup_sam4s.c \
$(ASF_DIR)/sam/drivers/pmc/pmc.c \
$(ASF_DIR)/sam/drivers/pio/pio.c \
$(ASF_DIR)/sam/drivers/twi/twi.c \
$(ASF_DIR)/sam/drivers/spi/spi.c \
$(ASF_DIR)/sam/drivers/wdt/wdt.c \
$(ASF_DIR)/sam/drivers/rstc/rstc.c \
$(ASF_DIR)/sam/drivers/tc/tc.c \
$(ASF_DIR)/common/services/clock/sam4s/sysclk.c \
$(ASF_DIR)/common/services/delay/sam/cycle_counter.c \
$(ASF_DIR)/common/services/sleepmgr/sam/sleepmgr.c             \
$(ASF_DIR)/common/services/usb/class/cdc/device/udi_cdc.c      \
$(ASF_DIR)/common/services/usb/class/cdc/device/udi_cdc_desc.c \
$(ASF_DIR)/common/services/usb/udc/udc.c                      \
$(ASF_DIR)/common/services/spi/sam_spi/spi_master.c           \
$(ASF_DIR)/common/utils/interrupt/interrupt_sam_nvic.c        \
$(ASF_DIR)/common/utils/stdio/read.c                          \
$(ASF_DIR)/common/utils/stdio/write.c                         \
$(ASF_DIR)/common/utils/stdio/stdio_usb/stdio_usb.c           \
$(ASF_DIR)/sam/drivers/pio/pio_handler.c                      \
$(ASF_DIR)/sam/drivers/sleep/sleep.c                          \
$(ASF_DIR)/sam/drivers/udp/udp_device.c                       \
$(ASF_DIR)/sam/drivers/pdc/pdc.c                              \
$(ASF_DIR)/sam/utils/syscalls/gcc/syscalls.c                  \

INCLUDEPATHS_COINES += \
.. \
. \
conf \
$(ASF_DIR)/sam/utils/cmsis/sam4s/include \
$(ASF_DIR)/sam/utils/cmsis/sam4s/include/component \
$(ASF_DIR)/sam/drivers/pmc \
$(ASF_DIR)/sam/drivers/pio \
$(ASF_DIR)/sam/drivers/twi \
$(ASF_DIR)/sam/drivers/spi \
$(ASF_DIR)/sam/drivers/wdt \
$(ASF_DIR)/sam/drivers/rstc \
$(ASF_DIR)/sam/drivers/tc \
$(ASF_DIR)/sam/drivers/udp  \
$(ASF_DIR)/sam/drivers/pdc  \
$(ASF_DIR)/thirdparty/CMSIS/Include \
$(ASF_DIR)/common/utils \
$(ASF_DIR)/sam/utils    \
$(ASF_DIR)/sam/utils/cmsis/sam4s/include  \
$(ASF_DIR)/sam/utils/header_files  \
$(ASF_DIR)/sam/utils/preprocessor   \
$(ASF_DIR)/common/boards \
$(ASF_DIR)/common/services/gpio \
$(ASF_DIR)/common/services/twi \
$(ASF_DIR)/common/services/clock \
$(ASF_DIR)/common/services/delay \
$(ASF_DIR)/common/services/ioport                             \
$(ASF_DIR)/common/services/spi/sam_spi                        \
$(ASF_DIR)/common/services/sleepmgr                           \
$(ASF_DIR)/common/services/usb                                \
$(ASF_DIR)/common/services/usb/class/cdc                      \
$(ASF_DIR)/common/services/usb/class/cdc/device               \
$(ASF_DIR)/common/services/usb/udc                            \
$(ASF_DIR)/common/utils/stdio/stdio_usb                       \
