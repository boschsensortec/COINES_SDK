# Firmware

## **Application Board 2.0**

### app2.0/DevelopmentDesktop_2.0_Firmware_V3.4.fwu2
- The default firmware preloaded in APP2.0
- This firmware was obsoleted and removed from app2.0 directory. It is recommended to use coines_bridge_flash_firmware instead

### app2.0/coines_bootloader/coines_usb_dfu_bl.pkg
- Required to run COINES examples directly on APP2.0 board's microcontroller

### app2.0/default_bootloader/Bootloader_APP2.0.bin
- Factory programmed default bootloader for APP2.0 board
- Supports USB and Bluetooth modes
- Flash this binary if you have accidentaly overwritten/erased the flash memory
- To flash `Bootloader_APP2.0.bin`,
  - Install [BOSSA](https://github.com/shumatech/BOSSA/releases) software and locate `bossac`
  - Short **J203** with jumper and power on the board to enter SAM-BA mode
  - Use the below command to flash the binary
```bash
$ bossac -e -w -v -b Bootloader_APP2.0.bin -U -p <com_port>
```

### app2.0/coines_bridge
- Enables the communication over serial
- Run `update_coines_bridge_fw.bat` to update coines_bridge firmware in APP2.0 board's flash location

---
## **Application Board 3.X**

### app3.0/DevelopmentDesktop_2.0_Firmware_v1.9.bin
- The default firmware preloaded in APP3.0
- This firmware was obsoleted and removed from app3.0 directory. It is recommended to use coines_bridge_flash_firmware instead
- APP3.1 was already preloaded with coines_bridge_flash_firmware

### app3.x/bootloader_update/usb_ble_dfu_bootloader.pkg
- Bootloader update package for APP3.X
- Comes preloaded on APP3.X board's microcontroller
- Run `update_bootloader.bat` script to update APP3.X bootloader (requires a atleast a old bootloader!)
- Power on the board with T2 button pressed to go to bootloader mode (Blue LED lights up)

### app3.x/mtp_fw_update/usb_mtp.pkg
- MTP firmware update package for APP3.X
- Comes preloaded on APP3.X board's microcontroller
- Required to read out files from APP3.X external flash memory (The board shows up as a USB storage device)
- Run `update_mtp_fw.bat` script to update APP3.X MTP firmware (requires the bootloader to be updated)
- Power on the board with T1 button pressed to go to MTP mode (Green LED lights up)
- Format the storage device for first use

### app3.x/coines_bridge
- Coines bridge firmware for APP3.X board
- Enables the communication over serial
- Run `update_coines_bridge_flash_fw.bat` to update coines_bridge firmware in APP3.X board's flash location
- Run `update_coines_bridge_ram_fw.bat` to update coines_bridge firmware in APP3.X board's RAM location

---
