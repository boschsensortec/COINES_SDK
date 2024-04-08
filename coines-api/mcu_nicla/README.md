# coinesAPI - NICLA MCU

## Notes

### coines_open_comm_intf

- The `coines_open_comm_intf` function initializes the nRF52832 microcontroller.
- It configures the CPU to run at 64 MHz.

### coines_close_comm_intf

The `coines_close_comm_intf` function de-initializes the LED driver.

### coines_config_i2c_bus

For the `coines_config_i2c_bus` function, 1.7 MHz and 3.4 MHz I2C is not supported

### coines_config_spi_bus

For the `coines_config_spi_bus` function, the SPI speed mapping is approximate.

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

The functions `printf`, `puts` work with USB serial.

### Switching to bootloader

To switch to bootloader:

- Press Button 1 three times for boot mode.

See the `app_switch` tool for more details.
