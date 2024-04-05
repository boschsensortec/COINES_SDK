# Command-line tool for flashing APP2.0 board and BNO USB stick

- Works with Windows, Linux and macOS
- Flash tool can be integrated into a `Makefile` project, shell script, etc.,

## Usage

``` bash
$ ./app20-flash firmware_file.fwu2
```
**Alternate way** : Drag 'n' drop the firmware file to `app20-flash` tool icon. 

## Linker script settings

| Hardware              | FLASH (Code)                  | RAM (Data)                       |
|-----------------------|:-----------------------------:|:--------------------------------:|
| Application Board 2.0 | 0x410000 - 0x500000 (960 kB)  | 0x20000000 - 0x20040000 (128 kB) |
| BNO USB stick         | 0x410000 - 0x420000 (64 kB)   | 0x20000000 - 0x20008000 (32 kB)  |

## Firmware update process

- **Bootloader mode** check (0x04)
-  Check **Board type** - APP2.0 board/BNO USB stick (0x09) 
- **Erase** command (0x01) 
- **Flash** command (0x02) 
- **Send firmware data** broken down as 50 byte packets (0x08) 
- **Update success** (0x07)

## Download COINES Bridge frimware to FLASH 

- Turn on board with switch-2 pressed.(Bootloader mode)

``` bash
$ app20-flash.exe coines_bridge_firmware.bin
```