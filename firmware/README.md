# Firmware
---
## **Application Board 3.0**
### app3.0/DevelopmentDesktop_2.0_Firmware_v1.9.bin
- The default firmware is preloaded in APP3.0.
- This firmware was obsoleted and removed from app3.0 directory. The user is required to use coines_bridge_flash_firmware instead.

### app3.0/bootloader_update/usb_ble_dfu_bootloader.pkg
- It has the bootloader update package for APP3.0.
- It comes preloaded on the APP3.0 board's microcontroller.
- Run the `update_bootloader` script to update the APP3.0 bootloader (requires a at least an old bootloader).
- Power on the board with the T2 button pressed to go to bootloader mode (Blue LED lights up).

### app3.0/mtp_fw_update/usb_mtp.pkg
- The **usb_mtp.pkg** contains the MTP firmware update package for APP3.0.
- It comes preloaded on the APP3.0 board's microcontroller.
- It's required to read out files from APP3.0 external flash memory (The board shows up as a USB storage device).
- Run the `update_mtp_fw` script to update the APP3.0 MTP firmware (requires the bootloader to be updated).
- Power on the board with T1 button pressed to go to MTP mode (the Green LED lights up).
- Format the storage device for first use.

### app3.0/coines_bridge
- Coines bridge firmware for App3.0 board.
- It enables the communication over the COM port.
- The user is required to use [Development Desktop 2.0](https://www.bosch-sensortec.com/software-tools/tools/development-desktop-software/), COINES_SDK and [coinespy](https://pypi.org/project/coinespy/).
- Run `update_coines_bridge_flash_fw.bat` to update the coines_bridge firmware in the App3.0 board's flash location.
- Run `update_coines_bridge_ram_fw.bat` to update the coines_bridge firmware in the App3.0 board's RAM location.


---
## **Application Board 3.1**
### app3.1/bootloader_update/usb_ble_dfu_bootloader.pkg
- It has the bootloader update package for APP3.1.
- It comes preloaded on the APP3.1 board's microcontroller.
- Run the `update_bootloader` script to update the APP3.1 bootloader (requires a at least an old bootloader).
- Power on the board with the S2 button pressed to go to bootloader mode (Blue LED lights up).

### app3.1/mtp_fw_update/usb_mtp.pkg
- The **usb_mtp.pkg** contains the MTP firmware update package for APP3.1.
- It comes preloaded on the APP3.1 board's microcontroller.
- It's required to read out files from APP3.1 external flash memory (The board shows up as a USB storage device).
- Run the `update_mtp_fw` script to update the APP3.1 MTP firmware (requires the bootloader to be updated).
- Power on the board with S3 button pressed to go to MTP mode (the Green LED lights up).
- Format the storage device for first use.

### app3.1/coines_bridge
- The default firmware is preloaded in APP3.1.
- It enables the communication over the COM port.
- The user is required to use [Development Desktop 2.0](https://www.bosch-sensortec.com/software-tools/tools/development-desktop-software/), COINES_SDK and [coinespy](https://pypi.org/project/coinespy/).
- Run `update_coines_bridge_flash_fw.bat` to update the coines_bridge firmware in the APP3.1 board's flash location.
- Run `update_coines_bridge_ram_fw.bat` to update the coines_bridge firmware in the APP3.1 board's RAM location.


---
## **Nicla**
### nicla/bootloader_update/ble_dfu_bootloader.hex
- It has the bootloader update package for Nicla.
- Run the `update_bootloader` script to update the bootloader (requires a at least an old bootloader).
- Press Button 1 three times to go to bootloader mode (Blue LED lights up).



### nicla/coines_bridge
- It enables the communication over the COM port.
- The user is required to use [Development Desktop 2.0](https://www.bosch-sensortec.com/software-tools/tools/development-desktop-software/), COINES_SDK and [coinespy](https://pypi.org/project/coinespy/).
- Run `update_coines_bridge_flash_fw.bat` to update the coines_bridge firmware in the Nicla board's flash location.

---

 