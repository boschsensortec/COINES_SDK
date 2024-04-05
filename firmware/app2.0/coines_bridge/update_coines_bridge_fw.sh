#!/bin/sh

# This script has not been tested on linux environment
# This script updates APP2.0 board with Coines Bridge firmware
# To update coines bridge FW, put the board to bootloader mode, press SW2 button and power on

CB_FW="coines_bridge_flash_firmware.bin"

if [ "$(strings $CB_FW | grep PATCHERE)" != "" ];
then
    echo "Fusing and patching firmware .."
	truncate -s 160K $CB_FW

	# Append USB DFU Bootloader
	cat usb_dfu_bootloader.bin  >> $CB_FW

	# Patch resultant firmware with USB DFU Bootloader start address
	sed -b -i 's/PATCHERE/\x00\x00\x00\x00\x00\x80\x43\x00/g' $CB_FW
	echo "Done !"
else
	echo "Already patched !"
fi

if ! [ -f "../../tools/app20-flash/app20-flash" ]; then
    make -C ../../tools/app20-flash
fi

../../../tools/app20-flash/app20-flash coines_bridge_flash_firmware.bin
