# Bosch Sensortec **COINES_SDK **
## **CO**mmunication with **IN**ertial and **E**nvironmental **S**ensors SDK

**COINES-SDK** allows users to evaluate sensors using the Bosch Sensortec Application Board. Sensor configuration and data readout can be easily done using the `coinesAPI` from PC side using C (or) Python.

 To overcome the limitations (Eg: inaccurate, delays,etc.,) due to latencies in USB communication, some C examples can also be cross-compiled and run directly on the Application Board's microcontroller with LOCATION=FLASH or LOCATION=RAM also.

## Quick Start
* Clone this repository
* Install GCC Toolchain and GNU Make
* Install USB drivers and libraries
  * Windows - app_board_usb_driver.exe
  * Linux 
    * `libusb-dev` package and `udev` rules
      * Debian based distros - `sudo apt install libusb-1.0-0-dev`
      * Red Hat based distros - `sudo yum install libusbx-devel`
    * `libdbus-1-dev` and `dbus-devel` packages
      * Debian based distros - `sudo apt install libdbus-1-dev`
      * Red Hat based distros - `sudo yum install dbus-devel`
  * macOS - `brew install libusb`
* Connect the Bosch Sensortec Application Board 2.0 (or) 3.X to PC with any sensor shuttle board mounted
* Go to any example and run `mingw32-make`(Windows) (or) `make`(Linux & macOS)
* Execute the compiled binary


## Running examples on Application Board microcontroller
* Update to the latest coines_bridge firmware
* Install [dfu-util](http://dfu-util.sourceforge.net)
  * Windows - Not required. Available at `tools/usb-dfu`
  * Linux
    * Debian based distros - `sudo apt install dfu-util`
    * Red Hat based distros - `sudo yum instal dfu-util`
  * macOS - `brew install dfu-util`
* Get [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads), extract and add to PATH environment variable
* Go to any example and run `make TARGET=MCU_APP20 download` (or) `make TARGET=MCU_APP30 download` (or) `make TARGET=MCU_APP31 download`
* Open Application Board USB serial port with any serial terminal program (Ensure DTR signal is asserted)

## Running Python examples
* Install Python 3.x
* Install library for each kind of CPU Architecture as below command set
  ```
  mingw32-make ARCH=x86_64 COINES_BACKEND=COINES_BRIDGE TARGET=PC
  ```
  (or)
  ```
  mingw32-make ARCH=x86 COINES_BACKEND=COINES_BRIDGE TARGET=PC
  ```
* Install `coinespy` by using any of the below command set
  ```bash
  $ pip install coinespy
  ```
  (or)
  ```bash
  $ cd coines-api/pc/python
  $ python setup.py install
  ```
* Run examples in `examples/python`

## Creating new examples

* For creating new examples, see `examples/c/template`
* Use `examples/c/template/Makefile` as a reference for including additional C, Assembly, C++ files and binary libraries.
