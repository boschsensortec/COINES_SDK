# `coinespy` library

With the `coinespy` library, users can access Bosch Sensortec's MEMS sensors on the Application Board 2.0 and Application Board 3.X through a Python interface. It offers a flexible solution for developing a host-independent sensor wrapper interface with a robust error-handling mechanism. The core functionalities remain the same as using coinesAPI on the C level.

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

To install this package, use the following code:
```bash
$ python setup.py install
```
To build a new wheel, follow the steps as described in: https://packaging.python.org/tutorials/packaging-projects/

To publish the python package, follow the steps as described in: [Packing python project](https://inside-docupedia.bosch.com/confluence/display/PST/Packaging+Python+Projects)