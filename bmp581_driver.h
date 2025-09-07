#pragma once
#ifndef MINIMAL_BMP581_H
#define MINIMAL_BMP581_H

#include <Arduino.h>
#include <Wire.h>

// Bosch BMP5 Sensor API
#include "bmp5.h"

// Default I2C addresses: SparkFun uses 0x47 by default (Bosch calls this "secondary")
// Alternate is 0x46. You can pass either to begin().
// (See SparkFun hookup/example docs for the board defaults.)
#ifndef BMP581_I2C_ADDR_DEFAULT
#define BMP581_I2C_ADDR_DEFAULT 0x47
#endif

class MinimalBMP581
{
public:
    MinimalBMP581();

    // Initialize over I2C. Address defaults to 0x47 (SparkFun).
    // Returns BMP5_OK (0) on success.
    int8_t begin(TwoWire &wire = Wire, uint8_t i2cAddr = BMP581_I2C_ADDR_DEFAULT);

    // Apply a very small set of defaults and start NORMAL mode.
    //Can pass a different ODR (see bmp5_defs.h) if you like.
    // Returns BMP5_OK (0) on success.
    int8_t configure(uint8_t odr = BMP5_ODR_25_HZ);

    // Read pressure (Pa) and temperature (°C).
    // Returns BMP5_OK (0) on success.
    int8_t read(float &pressure_pa, float &temperature_c);

    // Optional helpers
    int8_t standby();
    int8_t normal();

private:
    struct I2CIntf {
        TwoWire *wire;
        uint8_t addr;
    };

    // Static glue for Bosch API
    static BMP5_INTF_RET_TYPE i2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
    static BMP5_INTF_RET_TYPE i2cWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
    static void delayUs(uint32_t period, void *intf_ptr);

    bmp5_dev dev_{};
    I2CIntf i2c_{};
    bmp5_osr_odr_press_config cfg_{};
    bool ready_ = false;
};

#endif // MINIMAL_BMP581_H
