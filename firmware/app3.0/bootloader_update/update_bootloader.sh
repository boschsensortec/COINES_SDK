#!/bin/sh

# When bootloader update is complete, the board is switched to Application mode
if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

../../../tools/app_switch/app_switch usb_dfu_bl
dfu-util --device -,108c:ab3d -a RAM -D usb_ble_dfu_bootloader.pkg -R
