:: Red LED lights up when MTP firmware update is complete

@echo off
..\..\..\tools\app_switch\app_switch usb_dfu_bl
..\..\..\tools\usb-dfu\dfu-util --device -,108c:4b3d -a RAM -D usb_mtp.pkg -R
pause
 