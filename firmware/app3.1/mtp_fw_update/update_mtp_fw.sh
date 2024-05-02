#!/bin/sh

# Red LED lights up when MTP firmware update is complete

if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

../../../tools/app_switch/app_switch usb_dfu_bl
dfu-util --device -,108c:ab39 -a RAM -D usb_mtp.pkg -R
