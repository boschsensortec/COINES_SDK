# Generating a static (or) shared COINES_SDK library

## PC

### Static library with GCC
  1. Ensure you have installed the GCC toolchain (TDM/MinGW/Cygwin/Linux).
  2. Use `mingw32-make TARGET=PC`.
  3. The resulting library `libcoines-pc.a` and `coines.h` can be separately used in a project.

To link `application.c` with the COINES_SDK library, use the following command:

```
$ gcc application.c -I . -L . -lcoines-pc -lsetupapi
```
For Linux/macOS, replace `-lsetupapi` with `-lusb-1.0`.

-------------------------------------------------------------------------------

### Static/shared library with GCC/Clang/MSVC, etc.
* Windows: Get cmake from https://cmake.org/
* Ubuntu:

    ```
    $ sudo apt install cmake
    ```
* macOS:

    ```
    brew install cmake
    ```

```
$ cd coines-api/pc
$ mkdir build
$ cd build
$ cmake ..
```

(Execute `cmake` in *Developer Prompt for VS 201x/202x* in case of MSVC)

#### Visual Studio

```
$ msbuild coines-api.sln
```

Find `coines-pc.lib` and `coines.dll` in the coines-api/pc folder.

#### MinGW Makefiles

```
$ mingw32-make
```

Locate `libcoines-pc.a` and `libcoines.dll` in the coines-api/pc folder.

#### Linux/macOS

```
$ make
```

Go to the `libcoines-pc.a` and `libcoines.so`/`libcoines.dylib` (macOS) in `coines-api/pc` folder.

-------------------------------------------------------------------------------

## APP2.0/APP3.X/NICLA MCU:

1. Ensure you have installed the [GNU ARM Embedded Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).
2. Use `mingw32-make TARGET=MCU_APP20` (or) `mingw32-make TARGET=MCU_APP30` (or) `mingw32-make TARGET=MCU_APP31` (or) `mingw32-make TARGET=MCU_NICLA`.
3. The resulting library `libcoines-mcu_app20.a` (or) `libcoines-mcu_app30.a` (or) `libcoines-mcu_app31.a` (or) `libcoines-mcu_nicla.a` and `coines.h` can be separately used in a project.

#### NOTE
- The `libcoines-mcu_app20.a` and `libcoines-mcu_app30.a` and `libcoines-mcu_app31.a` have `printf` integration with USB CDC ACM 
- libcoines-mcu_nicla.a has `printf` integration with the USB Serial Bridge.
- File handling functions (`fopen`, `fclose`, `fprintf`, etc.) that can be used on the APP3.X flash memory are also integrated with `libcoines-mcu_app30.a` or `libcoines-mcu_app31.a`.
- APP3.X BLE (as Nordic UART service) can be used with the functions `fprintf`, `fwrite`, `fscanf`, `fread`, etc., over `bt_w` and `bt_r` files like `stdout`, `stderr`.
  ```c
  fprintf(bt_w, "Hello %d", 2020);
  fread(data, 1, 4, bt_r); // Reads 4-bytes from BLE
  ```

-------------------------------------------------------------------------------

Cygwin/Linux users can use `make` instead of `mingw32-make`.
