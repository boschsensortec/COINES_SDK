#!/bin/sh

# This script updates APP3.0 board with Coines Bridge firmware
if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

../../../tools/app_switch/app_switch usb_dfu_bl
dfu-util --device -,108c:ab39 -a RAM -D coines_bridge_ram_firmware.bin -R
