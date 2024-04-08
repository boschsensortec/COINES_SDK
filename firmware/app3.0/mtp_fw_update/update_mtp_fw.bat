:: When MTP update is complete, the board is switched to Application mode

@echo off
..\..\..\tools\app_switch\app_switch usb_dfu_bl
..\..\..\tools\usb-dfu\dfu-util --device -,108c:ab3d -a RAM -D usb_mtp.pkg -R
pause
 