#####################################################################################################
This example is intended to read the registers from the sensor which supports 16 bits wide registers

#####################################################################################################
Prerequisite
===========
Ensure that the Sensor supports 16 bit length registers

#####################################################################################################
Read sensor data
1. Initialize the application/board communication interface (USB) using COINES_SDK API
2. Initialize the sensor SPI interface
3. Wait for 200ms 
4. Read sensor using coines_read_16bit_spi() COINES_SDK API and print it on console for some iteration.
5. Close the communication interface
#####################################################################################################

