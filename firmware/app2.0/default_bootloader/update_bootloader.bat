:: 4 LEDs light up when bootloader update is complete
:: It is necessary to have atleast v3.1 of DD2.0 firmware to run this script
:: Press reset button after the update is done

@echo off
..\..\..\tools\app_switch\app_switch usb_dfu_bl
..\..\..\tools\usb-dfu\dfu-util --serial APP2.0-DFU -a RAM -D Bootloader_APP2.0.pkg -R
pause
