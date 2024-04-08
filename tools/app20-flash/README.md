# Command-line tool for flashing APP2.0 board and BNO USB stick

The command-line flash tool:
- works with Windows, Linux, and macOS
- can be integrated into a `Makefile` project, shell script, etc

## Use
**Method 1**:
``` bash
$ ./app20-flash firmware_file.fwu2
```
**Method 2**: 

Drag 'n' drop the firmware file to the `app20-flash` tool icon. 

## Linker script settings

| Hardware              | FLASH (Code)                  | RAM (Data)                       |
|-----------------------|:-----------------------------:|:--------------------------------:|
| Application Board 2.0 | 0x410000 - 0x500000 (960 kB)  | 0x20000000 - 0x20040000 (128 kB) |
| BNO USB stick         | 0x410000 - 0x420000 (64 kB)   | 0x20000000 - 0x20008000 (32 kB)  |

## Firmware update process
To update the firmware:

1. **Bootloader mode** check (0x04)
2. Check the **Board type** - APP2.0 board/BNO USB stick (0x09) 
3. **Erase** command (0x01) 
4. **Flash** command (0x02) 
5. **Send firmware data** broken down as 50-byte packets (0x08) 
6. **Update success** (0x07)

## Download Coines bridge firmware to FLASH 
To download the Coines bridge firmware to FLASH:

1. Turn on the App board by pressing switch-2 (Bootloader mode).
 
2. ``` bash
   $ app20-flash.exe coines_bridge_firmware.bin
   ```