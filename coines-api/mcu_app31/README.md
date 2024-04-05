# coinesAPI - APP3.1 MCU

## Quirky things

### coines_open_comm_intf

- Initializes nRF52840 microcontroller and waits indefinitely for serial port connection(DTR should be asserted) (or) BLE connection (or) T2 button to be pressed.
- Configures CPU to run at 64 MHz.
- Tries to mount filesystem in W25N02 NAND memory or does a clean format.

### coines_close_comm_intf

Flushes `stdout`/USB serial write buffer.

### coines_config_i2c_bus

- `SDO` pin made low to be consistent with PC coinesAPI
- 1.7 MHz and 3.4 MHz I2C is not supported

### coines_config_spi_bus
- SPI speed mapping is approximate

### Integration with standard C library
- `printf`, `puts` work with USB serial.
- `fopen`, `fclose`, `fprintf`, `fgets`, `remove` etc., work with filesystem on NAND flash

### Switching to bootloader or MTP mode 

Open and close USB serial port at 
 - 1200 baud for bootloader mode
 - 2400 baud for MTP mode.
 
See `app_switch` tool for more details.

Usage of APP2.0 pins in code intended for APP3.1 is permitted provided the shuttle EEPROM is loaded with APP2.0-APP3.1 pin mappings.
