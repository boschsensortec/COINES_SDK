# coinesAPI - APP3.1 MCU

## Notes

### coines_open_comm_intf

- The `coines_open_comm_intf` function initializes the nRF52840 microcontroller and waits indefinitely for a serial port connection (DTR should be asserted), (or) a BLE connection (or), the S2 button to be pressed.
- It configures the CPU to run at 64 MHz.
- It tries to mount the filesystem in W25N02 NAND memory or does a clean format.

### coines_close_comm_intf

The `coines_close_comm_intf` function flushes the `stdout`/USB serial write buffer.

### coines_config_i2c_bus

For the `coines_config_i2c_bus` function:

- The `SDO` (Serial Data Output) pin is made low to be consistent with the PC coinesAPI.
- 1.7 MHz and 3.4 MHz I2C is not supported.

### coines_config_spi_bus

- For the `coines_config_spi_bus` function, the SPI speed mapping is approximate.

### Unsupported APIs

The following APIs are not supported: 

- `coines_config_streaming`,
- `coines_start_stop_streaming`
- `coines_read_stream_sensor_data`
- `coines_trigger_timer`

Use the following APIs instead:

- `coines_attach_interrupt`
- `coines_detach_interrupt`

### Integration with the standard C library
- The functions `printf`, `puts` work with USB serial.
- The functions `fopen`, `fclose`, `fprintf`, `fgets`, `remove` etc., work with the filesystem on NAND flash.

### Switching to bootloader or MTP mode 
To switch to bootloader or MTP mode:

- Open and close USB serial port at: 
  - 1200 baud for bootloader mode
  - 2400 baud for MTP mode
 
See the `app_switch` tool for more details.
