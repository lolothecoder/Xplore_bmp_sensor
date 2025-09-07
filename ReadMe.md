# Xplore Pressure Sensor
## PCB Design and Schematics

### Connectors

I chose Molex connectors because they are reliable and don’t come loose easily. I’ve used them during my internship, so I know they perform well.

### Power Regulation

I used the AP63203WU buck converter for 5 V to 3.3 V regulation. It can handle up to 2 A.

Estimated current:\
- MCU: ~500 mA (at peak)\
- BMP581 sensor: ~300 µA\
- External device powered from the UART header: up to 500 mA (worst case)\
- Total worst-case current: ~1 A, so a 2 A buck converter gives plenty of margin.\
The wiring follows the datasheet:\
https://www.diodes.com/datasheet/download/AP63200-AP63201-AP63203-AP63205.pdf

I also added:

A diode on the input to prevent reverse current.
A 2.5 A fuse for overcurrent protection.

### MCU

I selected the ESP32-S3-WROOM-1 because:

I’m familiar with the Arduino environment.
It has integrated Wi-Fi, which can be useful for monitoring the sensor.

Key points:

Two GPIO pins with pull-up resistors for I2C.
USB_D+ and USB_D- wired for a USB-C connector.
Enable and Reset buttons wired like the official devkit:
https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-DevKitC-1_V1.1_20220413.pdf
These buttons allow proper boot and reset if the MCU hangs.
Added a heartbeat LED to check if the MCU is running.

### Sensor

The BMP581 sensor is wired with the default I2C address 0x47.
A decoupling capacitor is placed near the sensor to stabilize the 3.3 V supply.

### Status LEDs

I added LEDs for:

5 V
3.3 V
Heartbeat (MCU activity)

### USB-C

I used a USB-C connector for easy programming and testing before integrating into a larger system.
The wiring is based on this reference:
https://github.com/mike-rankin/ESP32-S3_MLX90640_Thermal/blob/main/PCB/Older_Rev2_Design/Schematic_Rev2.pdf

### Mounting

The board includes 4 mounting holes for easy installation.

## Code

The firmware is based on:

Bosch BMP5 SensorAPI
https://github.com/boschsensortec/BMP5_SensorAPI

SparkFun BMP581 Arduino Library
https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library/blob/main/src/SparkFun_BMP581_Arduino_Library.h
    
The driver includes:

Initialization and configuration functions.
I²C read/write functions.
A read function that returns pressure and temperature.

### Design Choices for Sensor Configuration
I²C Address

The BMP581 supports two addresses: 0x47 (secondary) and 0x46 (primary).
SparkFun boards and many breakout designs use 0x47 by default, so I kept that as the default in the driver.

OSR (Oversampling Ratio)

Pressure OSR (osr_p) = 4×
This reduces noise without making conversions too slow.
Temperature OSR (osr_t) = 2×
Enough for stable temperature compensation without wasting time.

ODR (Output Data Rate)

Default ODR = 25 Hz 
This gives a good balance between update speed and power consumption. It also easily fits the conversion time for the chosen OSR settings.
It gives plenty of time to gather the pressure and temperature values

These settings are a safe starting point for most applications and follow Bosch’s recommended practice:

Go to STANDBY, apply config, then switch to NORMAL mode for continuous measurements
