#!/bin/sh

# This script updates APP3.0 board with Coines Bridge firmware
if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

../../../tools/app_switch/app_switch usb_dfu_bl
dfu-util --device -,108c:4b3d -a FLASH -D coines_bridge_flash_firmware.bin -R
