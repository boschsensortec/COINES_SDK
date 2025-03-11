# BLE Support for the COINES_SDK PC module

## Supported platforms

**Windows**: Windows 10+

## Development

### Built with

SimpleBLE library v0.6.0: https://simpleble.readthedocs.io/en/latest/

### Tested with

- Windows 11 version 22H2
- Ubuntu 20.04.6 LTS
- MacOS Ventura 13.5.2

## Dependencies for Windows

### Added Dependencies

The following DLL files are already included in the **/coines-api/pc/coines_pc/platform/Windows/ble/simpleble-0.6.0/libs/x64** folder as a part of the coines source code:

- MSVCP140.dll
- VCRUNTIME140.dll
- VCRUNTIME140_1.dll

### Additional Dependencies

The following DLL files are additional dependencies:

- api-ms-win-crt-heap-l1-1-0.dll
- api-ms-win-crt-runtime-l1-1-0.dll
- api-ms-win-crt-math-l1-1-0.dll
- api-ms-win-crt-stdio-l1-1-0.dll
- api-ms-win-crt-locale-l1-1-0.dll
- api-ms-win-crt-string-l1-1-0.dll

### Dependency management

The COINES_SDK installer will check if the BLE dependencies are missing. If they are, the **/coines-api/pc/coines_pc/platform/Windows/ble/simpleble-0.6.0/bin** folder containing the required DLLs will be added to the COINES_SDK installation. Additionally, as part of the installation process, the bin path and SimpleBLE DLL paths will be set to system environmental variables.

## BLE APIs

The following APIs are for BLE:

> `ble_scan`: It connects to the BLE adapter and returns a list of BLE peripherals by initializing the dll load.

> `ble_connect`: It establishes a connection to the BLE peripheral using the provided BLE configurations.

> `ble_write`: It performs write operations using the BLE interface.

> `ble_read`: It sets a buffer with data received on BLE TX Notify.

> `ble_close`: It closes the BLE communication.

## Use

### General requirements

The Coines bridge firmware needs to be flashed to the App board by executing the following batch file:

> $ coines_2.0\firmware\app3.1\coines_bridge\update_coines_bridge_flash_fw.bat

To use the BLE module on a 64-bit operating system, it is necessary to use a 64-bit compiler such as MinGW64. Therefore, when building the executable file for the BLE examples, make sure to use the `mingw32-make` command with the 64-bit compiler installed at a path like "C:\msys64\mingw64\bin".
If user downloads COINES_SDK from GitHub, then user must add absolute path of folders below to system environment variables.
  - `coines-api\pc\coines_pc\platform\Windows\ble\simpleble-0.6.0\libs\x64`
  - `coines-api\pc\coines_pc\platform\Windows\ble\simpleble-0.6.0\bin\x64`

### Testing BLE communication

Change the BLE name and address in the scripts with your App board's BLE name and address.
You can find the BLE name of your App board by using the Bluetooth settings in Windows.

#### Using coines_bridge script - ble_com_test.c

1. $ cd coines_2.0\examples\c\common\ble_com_test\

2. $ mingw32-make

3. $ ble_com_test.exe

#### Using coinespy script - get_board_info.py and ble_config.py

Change the interface type to BLE in python files `get_board_info.py` and `ble_config.py`:

1. COM_INTF = cpy.CommInterface.BLE

2. $ cd coines_2.0\examples\python\

3. $ python get_board_info.py

Change the ble_com_config argument of the open_comm_interface API with the BLE name/address of your App board with the following script execution:

> $ python ble_config.py

### Usage with coines

Get a list of available BLE devices using the following code:

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

Establish communication with the App board via BLE using known BLE configurations:

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

### Use with coinespy

Establish communication with the App board via BLE with the following code:

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

BLE support for the Coines PC module has the following limitations:

- It does not support coines_flush_intf for BLE communication.
- It does not support data packeting for byte size greater than BLE payload size of 230.
- The BLE module was not tested with Windows 32-bit OS.
- The user might get an “Bluetooth not enabled” error code on Windows, despite Bluetooth being enabled. This can be fixed by using 64-bit compiler as mentioned in the **General requirements** section.
- "This is a known issue when running a version of SimpleBLE built for a 32-bit architecture on a 64-bit Windows machine":  https://simpleble.readthedocs.io/en/latest/simpleble/faq.html


## Quick fixes

If the App board fails to connect to the host via BLE, try any of the following steps:

- Disconnect and reconnect the App board.
- Reflash the App board with the coines_bridge firmware.
- Check if the environmental path for the simpleBLE DLL is set.
