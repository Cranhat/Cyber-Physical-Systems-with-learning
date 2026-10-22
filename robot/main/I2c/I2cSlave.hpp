#pragma once

#include <driver/i2c_master.h>
#include <cstdint>

class I2cSlave
{
public:
    I2cSlave(uint16_t address);
    ~I2cSlave();

    void setupSlave();

    i2c_device_config_t devConfig;
    i2c_master_dev_handle_t device;
    const uint16_t address;

private:
    const uint32_t sclSpeed = 100000;
};
