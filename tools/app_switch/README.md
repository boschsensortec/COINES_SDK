# Application Switch tool

The Application Switch tool can jump to different applications on the application board when specified with address (or) application name.

The tool has additional capability to switch from COINES example program to Bootloader mode (or) MTP mode.   

## Use

| Jump to ...                                        | Command                     |
|----------------------------------------------------|-----------------------------|
| Application at address `0x32000`                   | `$ ./app_switch 0x32000`    |
| USB DFU Bootloader of APP3.X                       | `$ ./app_switch usb_dfu_bl` |
|                                                    | `$ ./app_switch 0`          |
| APP3.X USB/BLE DFU Bootloader only                 | `$ ./app_switch 0xF0000`    |
| USB MTP firmware of APP3.X                         | `$ ./app_switch usb_mtp`    |
| USB MTP firmware of APP3.0                         | `$ ./app_switch 0x28000`    |
| USB MTP firmware of APP3.1                         | `$ ./app_switch 0xE3800`    |
|                                                    | `$ ./app_switch 0x440000`   |

## App switch using port info
### Windows
`$ ./app_switch <mode/address> <port name>` - E.g `$ ./app_switch 0 COM5`

### Linux/MacOS
`$ ./app_switch <mode/address> <serial number string>` - E.g `$ ./app_switch 0 D0F684FC766C`