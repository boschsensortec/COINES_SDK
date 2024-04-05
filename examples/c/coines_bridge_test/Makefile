# Root of COINES_SDK installation folder
# Can be C:/COINES/v2.0 , $(HOME)/COINES/v2.0 ,etc.,
# Don't use '\' 
COINES_INSTALL_PATH = ../../..

EXAMPLE_FILE = coines_bridge_test.c
# EXAMPLE_FILE = template.cpp

# Additional C files
# Eg: C_SRCS += support.c test.c
C_SRCS += ../../../coines-api/pc/coines_bridge.c
C_SRCS += ../../../coines-api/coines_common.c

# Additional C++ files
# Eg: CPP_SRCS += support.cpp test.cpp
CPP_SRCS += 

# Assembly source files. Take care of the processor architecture !
# Extension is '.S' not '.s'
# Eg: ASM_SRCS += support.S test.S
ASM_SRCS += 

# Additional paths to look for header files
# Eg: INCLUDEPATHS += inc inc/test
INCLUDEPATHS += ../../../coines-api/pc
INCLUDEPATHS += ../../../coines-api/pc/serial_com
INCLUDEPATHS += ../../../coines-api/pc/ble_com

# Additional C compiler flags
# Eg: CFLAGS += -D TEST=5 
CFLAGS +=

# Additional C++ compiler flags
# Eg: CPPFLAGS += -std=c++11 
CPPFLAGS +=

# Additional paths to look for library files
LIBPATHS +=

# Additional static/shared libraries
# Eg: To include libraries, libalgobsec.a and libalgobsx.a
# LIBS += algobsec algobsx
LIBS +=

COINES_BACKEND=COINES_BRIDGE

include $(COINES_INSTALL_PATH)/coines.mk
