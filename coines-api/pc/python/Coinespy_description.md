### `COINESPY`: Interface for Bosch Sensortec's Engineering Boards.
`COINESPY` provides a python interface for interacting with the Bosch Sensortec's Engineering boards.

- [GitHub repository](https://github.com/boschsensortec/COINES/tree/main/coines-api/pc/python)
- [Documentation](https://github.com/boschsensortec/COINES/blob/main/doc/BST-DHW-AN013.pdf)
- [Examples](https://github.com/boschsensortec/COINES/tree/main/examples/python)


The library offers the following range of functionalities:

- Control VDD and VDDIO of sensor
- Configure SPI and I2C bus parameters
- Read and write into registers of sensors from Bosch Sensortec via SPI and I2C
- Read and write digital pins of the Engineering Board.

### Engineering Board driver installation
To install the driver for the Engineering Board, follow these steps:
- Navigate to the USB driver location: `<path_to_python_site_packages>\coinespy\driver`.
- Run the USB driver.

### Firmware update
#### Prerequisites
Before proceeding, ensure that the selected Engineering Board is flashed with either `Coines bridge` or `DD` firmware.

#### Steps to update firmware
To update the `Coines bridge` firmware to the Engineering Board, follow these steps:
- Open the `firmware` folder located at `<path_to_python_site_packages>\coinespy\`.
- Select the appropriate subfolder that corresponds to your specific board type.
- Run the provided .bat/.sh file within the selected subfolder to initiate the firmware update process.

### Configuration for BLE communication
#### Windows
To enable Bluetooth Low Energy (BLE) communication with coinespy Python library on a Windows machine, users need to modify their environment variables after installing the package. Based on the PC's architecture, add one of the following paths to your environment variables.

- 64-bit: `<path_to_python_site_packages>\coinespy\bin\x64`
- 32-bit: `<path_to_python_site_packages>\coinespy\bin\x86`

#### Linux/MacOS
For BLE communication in Linux and MacOS, no additional configuration is necessary.

### Note

To determine the installation directory of coinespy, use `pip show coinespy`. Replace `<path_to_python_site_packages>` with your Python site packages directory.

### Code example

Here’s a script to verify the installation by fetching the COINESPY version, and the hardware and software versions of the connected board.

```python
	import coinespy as cpy
	from coinespy import ErrorCodes
	
	COM_INTF = cpy.CommInterface.USB
	
	if __name__ == "__main__":
		board = cpy.CoinesBoard()
		print('COINESPY version - %s' % cpy.__version__)
		board.open_comm_interface(COM_INTF)
		if board.error_code != ErrorCodes.COINES_SUCCESS:
			print(f'Could not connect to board: {board.error_code}')
		else:
			b_info = board.get_board_info()
			print(f"COINES SDK version: {board.lib_version}")
			print(
				f'BoardInfo: HW/SW ID: {hex(b_info.HardwareId)}/{hex(b_info.SoftwareId)}')
			board.close_comm_interface()
```

<br>