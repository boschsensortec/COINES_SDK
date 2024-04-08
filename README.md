# Bosch Sensortec **COINES_SDK**

## **CO**mmunication with **IN**ertial and **E**nvironmental **S**ensors SDK

With **CO**mmunication with **IN**ertial and **E**nvironmental **S**ensors (**COINES_SDK**), users can evaluate sensors using the Bosch Sensortec Application Board. Users can configure sensor settings and read data easily using the `coinesAPI` from a PC using C (or) Python.

To overcome the limitations (e.g., inaccurate delays,etc.) due to USB communication latencies, some C examples can also be cross-compiled and run directly on the Application Board's microcontroller.

## Quick Start

To get started with the Bosch Sensortec Application Board, follow these steps:
1. Clone the repository.
2. Install the GCC Toolchain and GNU Make on your system.
3. Install the USB drivers and libraries for your operating system:
* Windows: Run the app_board_usb_driver.exe
* Linux: 
  * Install the `libusb-dev` package and `udev` rules:
    * Debian-based distros: Run `sudo apt install libusb-1.0-0-dev`
    * Red Hat-based distros: Run `sudo yum install libusbx-devel`
  * Install the 'libdbus-1-dev' and 'dbus-devel' packages:
    * Debian-based distros: Run `sudo apt install libdbus-1-dev`
    * Red Hat-based distros: Run `sudo yum install dbus-devel` 
* macOS: use Homebrew to install libusb: Run `brew install libusb`
4. Connect the Bosch Sensortec Application Board to your PC with any sensor shuttle mounted.
5. Go to any example and run `mingw32-make`(Windows) (or) `make`(Linux & macOS) to compile the source code.
6. Run the compiled binary.

## Running examples on the Application Board microcontroller

To run examples on the Application Board microcontroller, follow these steps:
1. Update to the latest coines_bridge firmware
2. Install `dfu-util`. This is a utility used for uploading firmware to the microcontroller. The installation process varies depending on your operating system:
* Windows: No installation is required. `dfu-util` is available in the `tools/usb-dfu`folder.
* Linux:
  * Debian-based distros: Run `sudo apt install dfu-util`
  * Red Hat-based distros: Run `sudo yum instal dfu-util`
* macOS: Run `brew install dfu-util`
3. Get the [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads), extract and add to the PATH.
4. Go to any example and run `make TARGET=MCU_APP20 download` (or) `make TARGET=MCU_APP30 download` (or) `make TARGET=MCU_APP31 download.
5. Open the Application Board USB serial port with any serial terminal program (Ensure DTR signal is asserted).

## Running Python examples

To run Python examples, follow these steps:
1. Install Python 3.x, following the installation instructions provided on the website: <https://www.python.org/>
2. Install `coinespy` by using any of the below command set.

  ```bash
  pip install coinespy
  ```

  ```bash
  cd coines-api/pc/python
  mingw32-make ARCH=x86_64 
  python setup.py install
  ```
3. Run examples in the `examples/python` folder.

## Creating new examples

To create new examples, follow these steps:
1. Go to the `examples/c/template` folder.
2. Review the `Makefile` in this directory. This file contains instructions for compiling and linking the C, Assembly, C++ files, and binary libraries. Use it as a reference when adding your own files.
