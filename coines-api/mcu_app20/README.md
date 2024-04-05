# coinesAPI - APP2.0 MCU

## Quirky things

### coines_open_comm_intf

- Initializes SAM4S16C microcontroller and waits indefinitely for serial port connection (DTR should be asserted).
- Configures CPU to run at 60 MHz.
- Bluetooth is not supported. 

### coines_close_comm_intf

Flushes `stdout`/USB serial write buffer.

### coines_config_i2c_bus

`SDO` pin / `COINES_SHUTTLE_PIN_4` made low to be consistent with PC coinesAPI 

### Unsupported APIs

- `coines_config_streaming`
- `coines_start_stop_streaming`
- `coines_read_stream_sensor_data`
- `coines_trigger_timer`

Use the below APIs instead
- `coines_attach_interrupt`
- `coines_detach_interrupt`

### Integration with standard C library

- `printf`, `puts`, etc., work with USB serial.

### Switching to bootloader mode (secondary)

Open and close USB serial port at 1200 baud. 
See `app_switch` tool for more details.