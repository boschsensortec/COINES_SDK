@echo off
echo Starting APP3.1 firmware update process...
echo.

REM Set tool paths
set APP_SWITCH_PATH=..\..\tools\app_switch
set DFU_PATH=..\..\tools\usb-dfu

echo [1/3] Flashing COINES Bridge firmware...
%APP_SWITCH_PATH%\app_switch usb_dfu_bl
%DFU_PATH%\dfu-util --device -,108c:ab39 -a FLASH -D coines_bridge\coines_bridge_flash_firmware.bin -R
echo COINES Bridge firmware flashed successfully.
echo Waiting for device reset...
timeout /t 3 /nobreak >nul
echo.

echo [2/3] Flashing USB BLE DFU bootloader...
%APP_SWITCH_PATH%\app_switch usb_dfu_bl
%DFU_PATH%\dfu-util --device -,108c:ab39 -a RAM -D bootloader_update\usb_ble_dfu_bootloader.pkg -R
echo USB BLE DFU bootloader flashed successfully.
echo Waiting for device reset...
timeout /t 3 /nobreak >nul
echo.

echo [3/3] Flashing USB MTP firmware...
%APP_SWITCH_PATH%\app_switch usb_dfu_bl
%DFU_PATH%\dfu-util --device -,108c:ab39 -a RAM -D mtp_fw_update\usb_mtp.pkg -R
echo USB MTP firmware flashed successfully.
echo Waiting for device reset...
timeout /t 3 /nobreak >nul
echo.

echo -----------------------------------------------------------
echo APP3.1 firmware update process completed successfully!
echo All firmware components have been successfully flashed.
echo -----------------------------------------------------------
pause