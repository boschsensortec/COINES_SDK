#!/bin/sh

# 4 LEDs light up when bootloader update is complete
# It is necessary to have atleast v3.1 of DD2.0 firmware to run this script
# Press reset button after the update is done
if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

../../../tools/app_switch/app_switch usb_dfu_bl
dfu-util --serial APP2.0-DFU -a RAM -D Bootloader_APP2.0.pkg -R
