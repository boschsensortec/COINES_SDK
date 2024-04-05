#####################################################################################################
This example is intended to write and read back the 16 bit registers from the sensor which supports 16 bits wide registers

#####################################################################################################
Prerequisite
===========
Ensure that the Sensor supports 16 bit length registers

#####################################################################################################
Read sensor data
1. Initialize the application/board communication interface (USB) using COINES_SDK API
2. Initialize the sensor SPI interface
3. Wait for 200ms
4. Write 16bit values into the register using coines_write_16bit_spi() COINES_SDK API
5. Read back the register to verify if the write was done successfully
6. Close the communication interface
#####################################################################################################