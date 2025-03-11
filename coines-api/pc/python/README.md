# `coinespy` library

With the `coinespy` library, users can access Bosch Sensortec's MEMS sensors on the Application Board 3.X through a Python interface. It offers a flexible solution for developing a host-independent sensor wrapper interface with a robust error-handling mechanism. The core functionalities remain the same as using coinesAPI on the C level.

This **coinespy** folder contains the Python wrapper on top of the COINES C library.
* Dependencies libraries / software
   ```bash
   $ sudo apt-get install libdbus-1-dev
   $ sudo apt-get install libusb-1.0-0-dev
   $ sudo apt-get install libudev-dev
   $ sudo apt-get install gcc
   $ sudo apt-get install make
   $ sudo apt-get install dfu-util
   ```
* To build coines-api shared library, follow the steps as described below:
  * For MacOS arm: 
    ``` bash
    $ make ARCH=arm64
    ```
  * For Linux 64bit: 
    ``` bash
    $ make ARCH=x86_64
    ```
  * For Linux 32bit (should be build on system 32bit):
    ``` bash
    $ make ARCH=x86
    ```

  * For Windows 32bit: 
    ``` bash
    $ mingw32-make ARCH=x86
    ```
  * For Windows 64bit: 
    ``` bash
    $ mingw32-make ARCH=x86_64
    ```
# `Configurable parameter when generating library`
## Configure buffer size based on application need 
The following parameters allow configuring buffer size and queue depth as per application requirements.
```
CBUFF_SIZE=104857600  # 100MB  
MQUEUE_DEPTH=100  
```
## Streaming Support
FIFO streaming is only supported in interrupt streaming mode. The packet size should be configured based on application needs.
* Example Configurations:
  * Interrupt Streaming
  ```
  MQUEUE_PACKET_SIZE=255
  ```
  * FIFO Streaming - Watermark Level Calculation
    * Formula: Watermark Level = (Frame Length × Samples) + Packet Counter (4 Bytes) + Dummy Bytes (2 Bytes) + Ignore Bytes (Frame Header, if any - 2 Bytes)
    * Example Calculation: (18 × 30) + 4 + 2 + 2 = 548
  ```
  MQUEUE_PACKET_SIZE ?= 548  
  ```
  Note: Maximum supported streaming buffer size is 3KB.

## Timeout Configuration
Default values are configured inside the library, considering lower ODR (0.78Hz):
```
STREAM_RSP_TIMEOUT_MS = 1500  
FIFO_STREAM_RSP_TIMEOUT_MS = 10000  
```
## Custom Timeout Configuration
Timeout values can be adjusted based on application requirements:
```
STREAM_RSP_TIMEOUT ?= 0  # Milliseconds  
FIFO_STREAM_RSP_TIMEOUT ?= 0  # Milliseconds  
```
Note: If its zero default values inside the library will be utilized.

# Installation
To install this package, use the following code:
```bash
$ python setup.py install
```
To build a new wheel, follow the steps as described in: https://packaging.python.org/tutorials/packaging-projects/

To publish the python package, follow the steps as described in: [Packing python project](https://inside-docupedia.bosch.com/confluence/display/PST/Packaging+Python+Projects)
