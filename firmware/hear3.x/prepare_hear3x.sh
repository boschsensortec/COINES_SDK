#!/bin/bash

echo "Starting HEAR3.X firmware update process..."
echo

if ! [ -f "../../../tools/app_switch/app_switch" ]; then
    make -C ../../../tools/app_switch
fi

# Set tool paths
APP_SWITCH_PATH="../../tools/app_switch"

echo "[1/3] Flashing COINES Bridge firmware..."
$APP_SWITCH_PATH/app_switch usb_dfu_bl
dfu-util --device -,108c:4b3d -a FLASH -D coines_bridge/coines_bridge_flash_firmware.bin -R
echo "COINES Bridge firmware flashed successfully."
echo "Waiting for device reset..."
sleep 3
echo

echo "[2/3] Flashing USB BLE DFU bootloader..."
$APP_SWITCH_PATH/app_switch usb_dfu_bl
timeout 2
dfu-util --device -,108c:4b3d -a RAM -D bootloader_update/usb_ble_dfu_bootloader.pkg -R
echo "USB BLE DFU bootloader flashed successfully."
echo "Waiting for device reset..."
sleep 3
echo

echo "[3/3] Flashing USB MTP firmware..."
$APP_SWITCH_PATH/app_switch usb_dfu_bl
dfu-util --device -,108c:4b3d -a RAM -D mtp_fw_update/usb_mtp.pkg -R
echo "USB MTP firmware flashed successfully."
echo "Waiting for device reset..."
echo

echo "-----------------------------------------------------------"
echo "HEAR3.X firmware update process completed successfully!"
echo "All firmware components have been successfully flashed."
echo "-----------------------------------------------------------"
read -p "Press any key to continue..."