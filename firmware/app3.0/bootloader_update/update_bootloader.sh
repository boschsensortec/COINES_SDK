#!/bin/sh

# Green LED lights up when bootloader update is complete
# Switch 'OFF' and 'ON' the board after the update is done
if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

../../../tools/app_switch/app_switch usb_dfu_bl
dfu-util --device -,108c:ab3d -a RAM -D usb_ble_dfu_bootloader.pkg -R
