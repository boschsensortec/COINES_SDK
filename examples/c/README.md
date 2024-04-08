## Building and running C examples

|     Action                             |     Command                                             |
|:---------------------------------------|:--------------------------------------------------------|
| Compile for a PC (Default)               | `mingw32-make TARGET=PC`                                |
| Cross-compile for the APP2.0 board         | `mingw32-make TARGET=MCU_APP20`                         |
| Download the example to APP2.0 MCU RAM     | `mingw32-make LOCATION=RAM TARGET=MCU_APP20 download`   |
| Download the example to APP2.0 MCU FLASH   | `mingw32-make LOCATION=FLASH TARGET=MCU_APP20 download` |
| Download the example to APP3.0 MCU RAM     | `mingw32-make LOCATION=RAM TARGET=MCU_APP30 download`   |
| Download the example to APP3.0 MCU FLASH * | `mingw32-make LOCATION=FLASH TARGET=MCU_APP30 download` |
| Download the example to APP3.1 MCU RAM     | `mingw32-make LOCATION=RAM TARGET=MCU_APP31 download`   |
| Download the example to APP3.1 MCU FLASH * | `mingw32-make LOCATION=FLASH TARGET=MCU_APP31 download` |
| Run an example already residing in APP2.0 Flash memory | `mingw32-make run`                      |
| Clean the example                      | `mingw32-make clean`                                    |
| Clean the example and COINES library| `mingw32-make clean-all`                                |

Linux/MacOS/Cygwin/MSYS2 users can use `make`.


#### PC
On a PC, run the compiled executable:

```bash
$ ./example
```

#### Microcontroller

Use a Serial Terminal application to view the output:

- Windows: PuTTY, HTerm, etc.
- Linux: `cat` command (eg., `cat /dev/ttyACM0`)
- macOS: `screen` command (eg., `screen /dev/tty.usbmodem9F31`)

#### NOTES

- Downloading a COINES example to APP3.0 Flash memory will overwrite the default firmware.
- Some examples may not compile for both **PC** and **MCU** targets due to use of the POSIX C Library and APIs like `coines_config_streaming`, `coines_read_stream_sensor_data`, `coines_attach_interrupt`, etc.
- The binary on the MCU will be executed only when the serial port is opened and DTR is set. The program indefinitely waits at the function`coines_open_comm_intf(COINES_COMM_INTF_USB, NULL)` if DTR is not set. Some terminal programs such as **HTerm** allow explicit setting of the **DTR** signal.
