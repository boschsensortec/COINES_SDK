# coinesAPI - APP2.0 MCU

## Notes

### coines_open_comm_intf

- The `coines_open_comm_intf` function initializes the SAM4S16C microcontroller and waits indefinitely for a serial port connection (DTR should be asserted).
- It configures the CPU to run at 60 MHz.
- It does not support Bluetooth. 

### coines_close_comm_intf

The `coines_close_comm_intf` function flushes the `stdout`/USB serial write buffer.

### coines_config_i2c_bus

The `SDO` pin / `COINES_SHUTTLE_PIN_4` is made low to be consistent with the PC coinesAPI.

### Unsupported APIs
The following APIs are unsupported:

- `coines_config_streaming`,
- `coines_start_stop_streaming`
- `coines_read_stream_sensor_data`
- `coines_trigger_timer`

Instead, use the following APIs:
- `coines_attach_interrupt`
- `coines_detach_interrupt`

### Integration with the standard C library

The functions `printf`, `puts`, etc., work with a USB serial.

### Switching to bootloader mode (secondary)

To switch to bootloader mode (secondary):

1. Open and close the USB serial port at 1200 baud. 
2. See the `app_switch` tool for more details.