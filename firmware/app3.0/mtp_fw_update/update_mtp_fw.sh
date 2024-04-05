#!/bin/sh

# Green LED lights up when MTP firmware update is complete
# Switch 'OFF' and 'ON' the board after the update is done
if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

../../../tools/app_switch/app_switch usb_dfu_bl
dfu-util --device -,108c:ab3d -a RAM -D usb_mtp.pkg -R
