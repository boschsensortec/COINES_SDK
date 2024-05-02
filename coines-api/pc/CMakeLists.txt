cmake_minimum_required(VERSION 3.0)
project(coinesAPI)

set(SOURCE_FILES
coines.c
../coines_common.c
comm_intf/comm_intf.c
comm_intf/comm_ringbuffer.c
comm_driver/usb.c
)

set(INCLUDE_DIRECTORIES
.
..
comm_intf/
comm_driver/
comm_driver/libusb-1.0/
comm_driver/legacy_usb/
)

if (WIN32)
add_definitions(-DPLATFORM_WINDOWS -DLEGACY_USB_DRIVER -DAPP_START_ADDRESS=0x00030000)
list(APPEND SOURCE_FILES comm_driver/legacy_usb/legacy_usb_support.c)
endif()

if (UNIX)
add_definitions(-DPLATFORM_LINUX -DLIBUSB_DRIVER)
endif()

add_definitions(-DPC)

include_directories(${INCLUDE_DIRECTORIES})

add_library(coines-pc STATIC ${SOURCE_FILES})

add_library(coines SHARED ${SOURCE_FILES})

if (WIN32)
target_link_libraries(coines setupapi)
endif()

if (UNIX)
target_link_libraries(coines usb-1.0)
endif()

add_custom_command(TARGET coines-pc 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:coines-pc> ${CMAKE_SOURCE_DIR})

add_custom_command(TARGET coines 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:coines> ${CMAKE_SOURCE_DIR})
