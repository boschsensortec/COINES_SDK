:: Green LED lights up when MTP firmware update is complete
:: Switch 'OFF' and 'ON' the board after the update is done

@echo off
..\..\..\tools\app_switch\app_switch usb_dfu_bl
..\..\..\tools\usb-dfu\dfu-util --device -,108c:ab3d -a RAM -D usb_mtp.pkg -R
pause
 