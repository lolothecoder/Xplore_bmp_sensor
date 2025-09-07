#include "MinimalBMP581.h"

// -------- Public -------------------------------------------------------------

MinimalBMP581::MinimalBMP581() {}

int8_t MinimalBMP581::begin(TwoWire &wire, uint8_t i2cAddr)
{
    i2c_.wire = &wire;
    i2c_.addr = i2cAddr;

    //dev_ allows us to interract with the API
    dev_.intf = BMP5_I2C_INTF;
    dev_.read = &MinimalBMP581::i2cRead;
    dev_.write = &MinimalBMP581::i2cWrite;
    dev_.delay_us = &MinimalBMP581::delayUs;
    dev_.intf_ptr = &i2c_;
    dev_.handle = nullptr;

    // Datasheet recommends a soft reset after power-up; API provides one.
    // (bmp5_init will also handle basic device checks.)
    int8_t rslt = bmp5_init(&dev_);
    if (rslt != BMP5_OK) return rslt;

    ready_ = true;
    return BMP5_OK;
}

int8_t MinimalBMP581::configure(uint8_t odr)
{
    if (!ready_) return BMP5_E_NULL_PTR;

    int8_t rslt = BMP5_OK;

    //The Bosch example starts in a STANDBY mode
    // Put sensor in STANDBY before changing config
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &dev_);
    if (rslt != BMP5_OK) return rslt;

    // Start from current defaults then tweak minimal fields
    rslt = bmp5_get_osr_odr_press_config(&cfg_, &dev_);
    if (rslt != BMP5_OK) return rslt;

    //Enable pressure, modest OSR, chosen ODR
    cfg_.press_en = BMP5_ENABLE;                // enable pressure (temperature is on-chip for compensation)

    //OSR = Output Sample Rate (How many sample to avg); ODR = Output Data Rate (How many time to sample per sec)
    cfg_.osr_p    = BMP5_OVERSAMPLING_4X;       // modest noise reduction
    cfg_.osr_t    = BMP5_OVERSAMPLING_2X;       // minimum for good comp, still quick
    cfg_.odr      = odr;                        // default 25 Hz (change if desired)

    rslt = bmp5_set_osr_odr_press_config(&cfg_, &dev_);
    if (rslt != BMP5_OK) return rslt;

    // Low pass filter to smooth out spikes
    // a tiny IIR to tame spikes
    bmp5_iir_config iir{};
    iir.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
    iir.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
    iir.shdw_set_iir_p = BMP5_ENABLE;
    iir.shdw_set_iir_t = BMP5_ENABLE;

    //Even if it fails the code still runs since we're casting to void
    (void)bmp5_set_iir_config(&iir, &dev_); // ignore error for "minimum"

    // Enter NORMAL mode (continuous conversions at selected ODR)
    return bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &dev_);
}

int8_t MinimalBMP581::read(float &pressure_pa, float &temperature_c)
{

    //Only continue if the device is ready
    if (!ready_) return BMP5_E_NULL_PTR;

    bmp5_sensor_data data{};

    //Aquire data
    int8_t rslt = bmp5_get_sensor_data(&data, &cfg_, &dev_);
    pressure_pa   = data.pressure;
    temperature_c = data.temperature;
    if (rslt != BMP5_OK) return rslt;

    return BMP5_OK;
}

// To switch the state of the sensor
int8_t MinimalBMP581::standby()
{
    if (!ready_) return BMP5_E_NULL_PTR;
    return bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &dev_);
}

int8_t MinimalBMP581::normal()
{
    if (!ready_) return BMP5_E_NULL_PTR;
    return bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &dev_);
}

// -------- Private (Bosch API glue) ------------------------------------------
// To properly communicate with the sensor
BMP5_INTF_RET_TYPE MinimalBMP581::i2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if (!intf_ptr || !reg_data || length == 0) return BMP5_INTF_RET_ERROR;

    I2CIntf *ctx = reinterpret_cast<I2CIntf *>(intf_ptr);
    TwoWire *w = ctx->wire;

    w->beginTransmission(ctx->addr);
    w->write(reg_addr);
    if (w->endTransmission(false) != 0) return BMP5_INTF_RET_ERROR; // repeated start

    uint32_t read = w->requestFrom((int)ctx->addr, (int)length);
    if (read != length) return BMP5_INTF_RET_ERROR;

    for (uint32_t i = 0; i < length; i++) {
        reg_data[i] = (uint8_t)w->read();
    }

    return BMP5_INTF_RET_SUCCESS;
}

BMP5_INTF_RET_TYPE MinimalBMP581::i2cWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if (!intf_ptr || (!reg_data && length > 0)) return BMP5_INTF_RET_ERROR;

    I2CIntf *ctx = reinterpret_cast<I2CIntf *>(intf_ptr);
    TwoWire *w = ctx->wire;

    w->beginTransmission(ctx->addr);
    w->write(reg_addr);
    for (uint32_t i = 0; i < length; i++) {
        w->write(reg_data[i]);
    }

    return (w->endTransmission(true) == 0) ? BMP5_INTF_RET_SUCCESS : BMP5_INTF_RET_ERROR;
}

void MinimalBMP581::delayUs(uint32_t period, void * /*intf_ptr*/)
{
    // Arduino core provides delayMicroseconds for small waits
    delayMicroseconds((unsigned int)period);
}
