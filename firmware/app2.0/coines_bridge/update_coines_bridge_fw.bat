:: This script updates APP2.0 board with COINES Bridge firmware
:: To update put the board to bootloader mode, press SW2 button and power on

@echo off
setlocal
set MAXBYTESIZE=160000

FOR %%I in (coines_bridge_flash_firmware.bin) do set filesize=%%~zI

if %filesize% gtr %MAXBYTESIZE% (
    echo Already patched !
	echo.
	echo.
) else (
	:: Make CB Firmware of fixed size (160kB)
	truncate -s 160K coines_bridge_flash_firmware.bin
	:: Append `usb_dfu_bootloader.bin` (~5.1kB) to `coines_bridge_flash_firmware.bin` (160kB) which results in binary size of approximately 165 kB.
	cat fuse_and_patch_coines_bridge_fw/usb_dfu_bootloader.bin >> coines_bridge_flash_firmware.bin
	:: Patch to get the `Final coines_bridge_flash_firmware.bin` . Find the string `"PATCHERE"` and replace with `"\x00\x00\x00\x00\x00\x08\x43\x00"`
	sed -b -i 's/PATCHERE/\x00\x00\x00\x00\x00\x08\x43\x00/g' coines_bridge_flash_firmware.bin
	echo Combined COINES_SDK bootloader with COINES Bridge firmware
)

..\..\..\tools\app20-flash\app20-flash coines_bridge_flash_firmware.bin
pause