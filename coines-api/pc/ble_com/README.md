# BLE Support for COINES_SDK PC module

## Development

### Built With

SimpleBLE library v0.6.2 - https://simpleble.readthedocs.io/en/latest/

### Tested with

- Windows 11 version 22H2
- Ubuntu 20.04.6 LTS
- MacOS Ventura 13.5.2

## Dependencies for Windows

### Added Dependencies

The below DLL files are already included in the ble_com folder as a part of coines source code.

- MSVCP140.dll
- VCRUNTIME140.dll
- VCRUNTIME140_1.dll

### Additional Dependencies

- api-ms-win-crt-heap-l1-1-0.dll
- api-ms-win-crt-runtime-l1-1-0.dll
- api-ms-win-crt-math-l1-1-0.dll
- api-ms-win-crt-stdio-l1-1-0.dll
- api-ms-win-crt-locale-l1-1-0.dll
- api-ms-win-crt-string-l1-1-0.dll

### Dependency management

The COINES installer will check if the BLE dependencies are missing. If they are, the bin folder containing the required DLLs will be added to the COINES installation. Additionally, as part of the installation process, the bin path and simpleble DLL paths will be set to system environmental variables.

## BLE APIs

> `ble_scan`: Connects to BLE Adapter and returns list of BLE peripherals by initializing dll load.

> `ble_connect`: Establishes connection to BLE peripheral using the provided BLE configurations

> `ble_write`: Performs write operation using BLE interface

> `ble_read`: Sets buffer with data received on BLE TX Notify

> `ble_close`: Performs close operation of the BLE communication

## Usage

### General requirements

- Coines bridge firmware needs to flashed to App board by executing below batch file
  > $ coines_2.0\firmware\app3.1\coines_bridge\update_coines_bridge_flash_fw.bat
- To use the BLE module on a 64-bit operating system, it is necessary to use a 64-bit compiler such as MinGW64. Therefore, when building the executable file for the BLE examples, make sure to use the "mingw32-make" command with the 64-bit compiler installed at a path like "C:\msys64\mingw64\bin".
- If user downloads COINES directory from GitHub, then user must add absolute path of folders below to system environment variables.
  - `coines-api\pc\ble_com\simpleble-0.6.0\Windows\x64`
  - `coines-api\pc\ble_com\simpleble-0.6.0\Windows\bin\x64`

### Testing BLE communication

- Change the BLE name and address in the scripts with your App board's BLE name and address
- You could find the BLE name of your board by using Bluetooth settings in Windows.

#### Using coines_bridge script - ble_com_test.c

> $ cd coines_2.0\examples\c\ble_com_test\

> $ mingw32-make

> $ ble_com_test.exe

#### Using coinespy script - coinespy_test.py and coinespy_intf_args_test.py

Change the interface type to BLE in python files `coinespy_test.py` and `coinespy_intf_args_test.py`

> COM_INTF = cpy.CommInterface.BLE

> $ cd coines_2.0\examples\python\

> $ python coinespy_test.py

Change the ble_com_config argument of open_comm_interface API with BLE name/address of your board for below script execution

> $ python coinespy_intf_args_test.py

### Usage with coines

Get a list of available BLE devices using the below code

```
#include "coines.h"

struct ble_peripheral_info ble_info[40];
uint8_t peripheral_count;
size_t scan_timeout_ms = 7000;

/* Get the BLE peripheral list */
int16_t scan_result = coines_scan_ble_devices(ble_info, &peripheral_count, scan_timeout_ms);

/*
Do logic implementation with ble_info list
*/
```

Establish communication with App board via BLE using known BLE configurations

```
struct ble_peripheral_info ble_config = {BLE_ADDR, BLE_NAME};
coines_open_comm_intf(COINES_COMM_INTF_BLE, &ble_config);

/*
Perform sensor read and write operations
*/

coines_close_comm_intf(COINES_COMM_INTF_BLE, NULL);
```
(OR)

Establish BLE communication with no configurations. In this case, the host will be connected to the nearest APP board.

```
coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);

/*
Perform sensor read and write operations
*/

coines_close_comm_intf(COINES_COMM_INTF_BLE, NULL);
```

### Usage with coinespy

Establish communication with App board via BLE

```
import coinespy as cpy

board = cpy.CoinesBoard()

ble_com_config = cpy.BleComConfig()
ble_com_config.identifier = BLE_NAME
ble_com_config.address = BLE_ADDR

board.open_comm_interface(cpy.CommInterface.BLE, ble_com_config=ble_com_config)

/* Comm connection without config as shown below
/* board.open_comm_interface(cpy.CommInterface.BLE)

/*
Perform sensor read and write operations
*/

board.close_comm_interface()
```

## Limitations

1. Does not support coines_flush_intf for BLE communication
2. Does not support data packeting for byte size greater than BLE payload size of 230.
3. BLE module was not tested with Windows 32-bit OS
4. User might get “Bluetooth not enabled” error code on Windows, despite Bluetooth being enabled. This can be fixed by using 64-bit compiler as mentioned in General requirements section.
   > "This is a known issue when running a version of SimpleBLE built for a 32-bit architecture on a 64-bit Windows machine" - https://simpleble.readthedocs.io/en/latest/simpleble/faq.html


## Quick fixes

If App board fails to connect to host via BLE, please try any of the below steps

- Disconnect and reconnect App board
- Reflash the App board with coines_bridge firmware
- Check if environmental path for simpleble DLL is set on Windows machine
