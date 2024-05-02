# APP3.0 BLE DFU Tool

The `app30-ble-dfu` tool is a PC-based tool written in Python to perform Device Firmware Upgrade (DFU) using native BLE hardware.

This tool works with the Bosch Sensortec APP3.0 board, nRF5 devices with [Adafruit nRF52 bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader), and the bootloader from Nordic SDK v11 and below.

It works with the latest Bluetooth v4.0 USB dongles and recent notebook PCs with Bluetooth. It was tested with the CSR8510 dongle in Windows 10 (Build 16299 and above), Windows 11 (Build 22621 and above) and Ubuntu 20.04 LTS.

```
C:\>python app30-ble-dfu.py -d <device_mac_address> -f <firmware_binary_file>
```

---

## Use

### Requirements
- [Python 3](https://www.python.org/downloads/)
- [bleak](https://pypi.org/project/bleak/) library

### Install dependencies in `requirements.txt`
```
$ pip install -r requirements.txt
```
### Scan for BLE devices
```
C:\Tools> python app30-ble-dfu.py -l

Application Board 3.0 BLE DFU tool
Bosch Sensortec GmbH (C) 2024

Scanning for devices...
APP3.0 Board (DFU)        D7:A3:CE:8E:36:14      -56 dB
APP3.0 Board (DFU)        F5:0C:EF:B7:D5:35      -63 dB
Unknown                   C8:28:32:E1:39:CA      -68 dB
Unknown                   EA:23:43:92:06:15      -57 dB
```

### Perform DFU

```
C:\Tools> python app30-ble-dfu.py -d D7:A3:CE:8E:36:14 -f nrf52_ble_basic.bin
Application Board 3.0 BLE DFU tool
Bosch Sensortec GmbH (C) 2024

Connected to D7:A3:CE:8E:36:14
nRF5 BLE DFU service found - v0.8

Download  [=========================]  100 % (28.07 kB/28.07 kB)

DFU Success !
```

### Show help

```
C:\Tools> python app30-ble-dfu.py --help
Application Board 3.0 BLE DFU tool
Bosch Sensortec GmbH (C) 2024

usage: app30-ble-dfu.py [-h] [-l] [-d DEVICE_MAC_ADDR] [-f FIRMWARE_BIN_FILE]

optional arguments:
  -h, --help            show this help message and exit
  -l, --list            Scan for BLE devices

required arguments:
  -d DEVICE_MAC_ADDR    Specify device MAC address
  -f FIRMWARE_BIN_FILE  Specify firmware binary
```
---

## Background information

BLE in Windows 7 or 8 required the use of a specialized dongle like [Silicon Labs BLED112](https://www.silabs.com/wireless/bluetooth/bluegiga-low-energy-legacy-modules/device.bled112), [TI CC2540 BLE USB](http://www.ti.com/tool/TIDC-CC2540-BLE-USB), [Nordic nRF51 dongle](https://www.nordicsemi.com/Software-and-tools/Development-Kits/nRF51-Dongle), etc ., which is locked to vendor-specific software.

With Windows 10 Fall Creator's Update (Build 16299/Version 1709), BLE is supported natively. This tool pushes native Bluetooth hardware built into notebook PCs and USB dongles without getting locked into vendor-specific hardware and software.

---