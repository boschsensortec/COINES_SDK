# COINES Bridge version info
COINES_BRIDGE_VERSION_MAJOR = 1
COINES_BRIDGE_VERSION_MINOR = 7
COINES_BRIDGE_VERSION_PATCH = 4

# Root of COINES_SDK installation folder
# Can be C:/COINES/v2.0 , $(HOME)/COINES/v2.0 ,etc.,
# Don't use '\' 
COINES_INSTALL_PATH = ../../..

EXAMPLE_FILE = coines_bridge_firmware.c
# EXAMPLE_FILE = template.cpp

# Additional C files
# Eg: C_SRCS += support.c test.c
C_SRCS += coines_bridge_client.c
C_SRCS += coines_bridge_stream.c
C_SRCS += decoder/decoder.c
C_SRCS += stream/decoder_support.c
C_SRCS += stream/stream.c
C_SRCS += $(COINES_INSTALL_PATH)/libraries/job_queue/job_queue.c
C_SRCS += $(COINES_INSTALL_PATH)/libraries/mbuf/mbuf.c


# Additional C++ files
# Eg: CPP_SRCS += support.cpp test.cpp
CPP_SRCS += 

# Assembly source files. Take care of the processor architecture !
# Extension is '.S' not '.s'
# Eg: ASM_SRCS += support.S test.S
ASM_SRCS += 

# Additional paths to look for header files
# Eg: INCLUDEPATHS += inc inc/test
INCLUDEPATHS += decoder 
INCLUDEPATHS += stream 
INCLUDEPATHS += $(COINES_INSTALL_PATH)/libraries/nrf52_eeprom 
INCLUDEPATHS += $(COINES_INSTALL_PATH)/libraries/job_queue
INCLUDEPATHS += $(COINES_INSTALL_PATH)/libraries/mbuf

# Default read buff size
READ_BUFF_SIZE ?= 3072 

# Additional C compiler flags
# Eg: CFLAGS += -D TEST=5 
CFLAGS += -DCOM_READ_BUFF_SIZE=$(READ_BUFF_SIZE) 
CFLAGS += -DSOFTWARE_VERSION_MAJOR=$(COINES_BRIDGE_VERSION_MAJOR) 
CFLAGS += -DSOFTWARE_VERSION_MINOR=$(COINES_BRIDGE_VERSION_MINOR) 
CFLAGS += -DSOFTWARE_VERSION_PATCH=$(COINES_BRIDGE_VERSION_PATCH)

# Additional C++ compiler flags
# Eg: CPPFLAGS += -std=c++11 
CPPFLAGS +=

# Additional paths to look for library files
LIBPATHS +=

# Additional static/shared libraries
# Eg: To include libraries, libalgobsec.a and libalgobsx.a
# LIBS += algobsec algobsx
LIBS +=

LOCATION=FLASH

include $(COINES_INSTALL_PATH)/coines.mk