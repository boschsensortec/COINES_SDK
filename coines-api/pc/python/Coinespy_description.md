### `COINESPY`: Interface for Bosch Sensortec's Engineering Boards.
`COINESPY` provides a python interface for interacting with the Bosch Sensortec's Engineering boards.

- [GitHub repository](https://github.com/boschsensortec/COINES/tree/main/coines-api/pc/python)
- [Documentation](https://boschsensortec.github.io/COINES/py_api/coinespy_api_description/coinespy_api_description/)
- [Examples](https://github.com/boschsensortec/COINES/tree/main/examples/python)


The library offers the following range of functionalities:

- Control VDD and VDDIO of sensor
- Configure SPI and I2C bus parameters
- Read and write into registers of sensors from Bosch Sensortec via SPI and I2C
- Read and write digital pins of the Application Board.

### Code example

Here’s a script to verify the installation by fetching the coinespy version, and the hardware and software versions of the connected board.

```python
	import coinespy as cpy
	from coinespy import ErrorCodes
	
	COM_INTF = cpy.CommInterface.USB
	
	if __name__ == "__main__":
		board = cpy.CoinesBoard()
		print('coinespy version - %s' % cpy.__version__)
		board.open_comm_interface(COM_INTF)
		if board.error_code != ErrorCodes.COINES_SUCCESS:
			print(f'Could not connect to board: {board.error_code}')
		else:
			b_info = board.get_board_info()
			print(f"coines lib version: {board.lib_version}")
			print(
				f'BoardInfo: HW/SW ID: {hex(b_info.HardwareId)}/{hex(b_info.SoftwareId)}')
			board.close_comm_interface()
```

<br>
