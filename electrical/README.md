# FIMUR Electrical
## Naming Convention
Each board has a 3-digit version number: A.B.C
A - major version number
B - board type. 
    1 is ICM20948 i2c breakout;
    2 is i2c multiplexer breakout and esp32 interface
C - minor version number

## IMC20948 Breakout (x.1.x)
I2C Breakout for two ICM20948 9-axis IMUs

### Exposes
- I2C bus
    two connectors to allow daisychaining
- pullup pins
    One and only one device on the bus must jump the pullup pins to allow for correct logic operation.
- FSYNC pins
    Future Work: should be able to daisychain all IMUs on this line to synchronize sample times

### v0.1.1
Works. Should add some capacitors to the 1.8V power supply as per spec sheet.

## Multiplexer Interface (x.2.x)
Connects ESP32 with TCA9548A I2C multiplexer.

### Exposes
- i2c bus upstream of multiplexer
    two connectors to allow daisychaining
- pullup pins for upstream i2c bus
    One and only one device on the upstream bus must jump the pullup pins to allow for correct logic operation.
- address pins for multiplexer
    each multiplexer on the upstream i2c bus must have a unique address. Addresses range from 0x70 to 0x77. The default address w/o any address pins driven is 0x70.
- eight multiplexed i2c channels downstream of the multiplexer. 
    These act as their own bus, so devices with the same address can be used if on different channels.

The board uses 3.3V I2C logic supplied directly from the ESP32.

### 0.2.1
Bad: made ESP32 get very hot. Do not use.

### 0.2.2
Fixed issues with 0.2.1. Tested with two IMUs connected to the same and differet channels.
