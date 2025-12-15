:: Mass chip erase
:: Write bootloader start address
:: Flash softdevice

@echo off
echo Starting Nicla firmware update process...
echo.

REM Set tool paths
set OPEN_OCD= ..\..\tools\openocd\xpack-openocd-0.11.0-4\bin\openocd
set LIB_PATH= ..\..\coines-api\pc\comm_driver\libusb-1.0\mingw_lib

echo [1/5] Performing mass chip erase...
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface\cmsis-dap.cfg -f target\nrf52.cfg  -c "init; reset halt;" -c "init; nrf5 mass_erase;" -c "reset init;" -c "shutdown;"
echo Mass chip erase completed successfully.
echo.

echo [2/5] Writing bootloader start address...
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface\cmsis-dap.cfg -f target\nrf52.cfg -c "init; reset halt;" -c "init; flash erase_sector 1 0 last;" -c "init; flash fillw 0x10001014 0x26000 1;" -c "reset init;" -c "shutdown;"
echo Bootloader start address written successfully.
echo.

echo [3/5] Flashing SoftDevice...
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface\cmsis-dap.cfg -c "transport select swd; adapter speed 1000" -f target/nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter speed 10000;" -c "program softdevice/s132_nrf52_7.2.0_softdevice.hex" -c "reset run; shutdown"
echo SoftDevice flashed successfully.
echo.

echo [4/5] Flashing BLE DFU bootloader...
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface/cmsis-dap.cfg -c "transport select swd; adapter speed 1000" -f target/nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter speed 10000;" -c "program bootloader_update/ble_dfu_bootloader.hex" -c "reset run; shutdown"
echo BLE DFU bootloader flashed successfully.
echo.

echo [5/5] Flashing COINES Bridge firmware...
%OPEN_OCD% -d2 -s %LIB_PATH% -f interface/cmsis-dap.cfg -c "transport select swd; adapter speed 1000" -f target/nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter speed 10000;" -c "program coines_bridge/coines_bridge_flash_firmware.hex" -c "reset run; shutdown"
echo COINES Bridge firmware flashed successfully.
echo.

echo -----------------------------------------------------------
echo Nicla firmware update process completed!
echo All components have been flashed to the device.
echo -----------------------------------------------------------

pause