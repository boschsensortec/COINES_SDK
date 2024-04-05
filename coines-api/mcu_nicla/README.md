# coinesAPI - NICLA MCU

## Quirky things

### coines_open_comm_intf

- Initializes nRF52832 microcontroller.
- Configures CPU to run at 64 MHz.

### coines_close_comm_intf

De-initializes LED driver.

### coines_config_i2c_bus

- 1.7 MHz and 3.4 MHz I2C is not supported

### coines_config_spi_bus
- SPI speed mapping is approximate

### Unsupported APIs

- `coines_config_streaming`,
- `coines_start_stop_streaming`
- `coines_read_stream_sensor_data`
- `coines_trigger_timer`

Use the below APIs instead
- `coines_attach_interrupt`
- `coines_detach_interrupt`

### Integration with standard C library
- `printf`, `puts` work with USB serial-bridge.

### Switching to bootloader

Press Button 1 three times for boot mode

See `app_switch` tool for more details.
