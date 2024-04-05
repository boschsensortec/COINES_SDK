:: Red LED lights up when bootloader update is complete

@echo off
..\..\..\tools\app_switch\app_switch usb_dfu_bl
..\..\..\tools\usb-dfu\dfu-util --device -,108c:ab39 -a RAM -D usb_ble_dfu_bootloader.pkg -R
pause
 