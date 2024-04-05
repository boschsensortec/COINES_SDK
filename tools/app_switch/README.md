# Application Switch tool

Tool to jump to different applications on Application Board when specified with address (or) application name.

The tool has additional tricks to switch from COINES example program to Bootloader mode (or) MTP mode.   

## Usage

| Jump to ...                                        | Command                     |
|----------------------------------------------------|-----------------------------|
| Application at address `0x28000`                   | `$ ./app_switch 0x28000`    |
| USB DFU Bootloader of APP2.0 or APP3.X             | `$ ./app_switch usb_dfu_bl` |
|                                                    | `$ ./app_switch 0`          |
| APP2.0 USB DFU Bootloader only                     | `$ ./app_switch 0x438000`   |
| APP3.X USB/BLE DFU Bootloader only                 | `$ ./app_switch 0xF0000`    |
| USB MTP firmware of APP3.X                         | `$ ./app_switch usb_mtp`    |
|                                                    | `$ ./app_switch 0x28000`    |
| COINES example residing on Flash memory of APP2.0  | `$ ./app_switch example`    |
|                                                    | `$ ./app_switch 0x440000`   |


## COM port open/close trick

For this trick to work, `COINES example` should be running on the Application Board microcontroller. 

Opening Application Board's COM port at 1200 baud of puts the board to Bootloader mode. `COINES example` contains code to read this trick.

> Credits to Arduino for the [concept](https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/CDC.cpp#L101)
> 
>  [Arduino Leonardo](https://www.arduino.cc/en/Main/Arduino_BoardLeonardo) - See **Automatic (Software) Reset and Bootloader Initiation**


### Windows implementation

- Find COM port no. with USB VID (0x108C) and PID (0xAB2C/0xAB3C/0xAB38) using `SetupAPI`
- Open and close COM port at baud rate of 1200 bps with DTR asserted

### Linux/macOS implementation

Uses libUSB to perform open and close COM port at 1200 bps since it is the simplest. This method can also snatch control from serial terminal program for accessing serial port directly(No need of closing serial port). `udev` rule needs to added for Linux.

- Open USB device by VID and PID
- By means of USB control transfer
  - Set CDC control line state - DTR & RTS
    - bmRequestType = 0x21
    - bRequest = 0x22
    - wValue = 0x0003  <--- DTR | RTS  = (1<< 1) | (1 << 0)
    - wIndex = 0x0000
    - wLength = 0x0000
  - Set CDC Line coding  - 1200,8N1
    - bmRequestType = 0x21
    - bRequest = 0x20
    - wValue = 0x0000
    - wIndex = 0x0000
    - wLength = 0x0007
    - Data =  { 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x08 } <--- `1200 8N1 , 1200 = 0x000004B0`
  - Clear control line state - DTR & RTS
    - bmRequestType = 0x21
    - bRequest = 0x22
    - wValue = 0x0000  <--- ~(DTR | RTS)
    - wIndex = 0x0000
    - wLength = 0x0000

For MTP mode on APP3.X board, the above actions are performed at 2400 baud.

