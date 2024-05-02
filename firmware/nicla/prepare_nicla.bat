:: Mass chip erase
:: Write bootloader start address
:: Flash softdevice

set OPEN_OCD= ..\..\tools\openocd\xpack-openocd-0.11.0-4\bin\openocd
set LIB_PATH= ..\..\coines-api\pc\comm_driver\libusb-1.0\mingw_lib
@echo off

%OPEN_OCD% -d2 -s %LIB_PATH% -f interface\cmsis-dap.cfg -f target\nrf52.cfg  -c "init; reset halt;" -c "init; nrf5 mass_erase;" -c "reset init;" -c "shutdown;"
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface\cmsis-dap.cfg -f target\nrf52.cfg -c "init; reset halt;" -c "init; flash erase_sector 1 0 last;" -c "init; flash fillw 0x10001014 0x26000 1;" -c "reset init;" -c "shutdown;"
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface\cmsis-dap.cfg -c "transport select swd; adapter speed 1000" -f target\nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter speed 10000;" -c "program softdevice/s132_nrf52_7.2.0_softdevice.hex" -c "reset run; shutdown"
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface/cmsis-dap.cfg -c "transport select swd; adapter speed 1000" -f target/nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter speed 10000;" -c "program bootloader_update/ble_dfu_bootloader.hex" -c "reset run; shutdown"
echo -----------------------------------------------------------

pause