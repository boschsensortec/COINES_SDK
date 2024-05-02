# Application Switch tool

The Application Switch tool can jump to different applications on the application board when specified with address (or) application name.

The tool has additional capability to switch from COINES example program to Bootloader mode (or) MTP mode.   

## Use

| Jump to ...                                        | Command                     |
|----------------------------------------------------|-----------------------------|
| Application at address `0x28000`                   | `$ ./app_switch 0x28000`    |
| USB DFU Bootloader of APP2.0 or APP3.X            | `$ ./app_switch usb_dfu_bl` |
|                                                    | `$ ./app_switch 0`          |
| APP2.0 USB DFU Bootloader only                     | `$ ./app_switch 0x438000`   |
| APP3.X USB/BLE DFU Bootloader only                 | `$ ./app_switch 0xF0000`    |
| USB MTP firmware of APP3.X                         | `$ ./app_switch usb_mtp`    |
|                                                    | `$ ./app_switch 0x28000`    |
| COINES example residing on Flash memory of APP2.0  | `$ ./app_switch example`    |
|                                                    | `$ ./app_switch 0x440000`   |


## COM port open/close 
 The `COINES example` should be running on the application board microcontroller. 

Opening the application board's COM port at 1200 baud puts the board in Bootloader mode. The `COINES example` contains code to read this condition.

> [concept](https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/CDC.cpp#L101)


### Windows implementation

1. Find the COM port no. with USB VID (0x108C) and PID (0xAB2C/0xAB3C/0xAB38) using `SetupAPI`.
2. Open and close the COM port at the baud rate of 1200 bps with DTR asserted.

### Linux/macOS implementation

Since the Linux/macOS implementation is the simplest method, it uses libUSB to open and close the COM port at 1200 bps. This method can also take control from the serial terminal program to access the serial port directly (No need to close the serial port). Add the `udev` rule for Linux.

1. Open the USB device by VID and PID.
2. By means of USB control transfer:
   1. Set CDC control line state - DTR & RTS:
      - bmRequestType = 0x21
      - bRequest = 0x22
      - wValue = 0x0003  <--- DTR | RTS  = (1<< 1) | (1 << 0)
      - wIndex = 0x0000
      - wLength = 0x0000
   2. Set CDC Line coding  - 1200,8N1:
      - bmRequestType = 0x21
      - bRequest = 0x20
      - wValue = 0x0000
      - wIndex = 0x0000
      - wLength = 0x0007
      - Data =  { 0xB0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x08 } <--- `1200 8N1 , 1200 = 0x000004B0`
   3. Clear control line state - DTR & RTS:
      - bmRequestType = 0x21
      - bRequest = 0x22
      - wValue = 0x0000  <--- ~(DTR | RTS)
      - wIndex = 0x0000
      - wLength = 0x0000

For MTP mode on the APP3.X board, the actions above are performed at 2400 baud.

