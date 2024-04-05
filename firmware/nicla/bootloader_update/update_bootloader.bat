:: Flash Bootloader
set OPEN_OCD= ..\..\..\tools\openocd\xpack-openocd-0.11.0-4\bin\openocd
set LIB_PATH= ..\..\..\coines-api\pc\comm_driver\libusb-1.0\mingw_lib
@echo off

%OPEN_OCD% -d2 -s %LIB_PATH% -f interface/cmsis-dap.cfg -c "transport select swd; adapter speed 1000" -f target/nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter speed 10000;" -c "program ble_dfu_bootloader.hex" -c "reset run; shutdown"
echo -----------------------------------------------------------------
pause