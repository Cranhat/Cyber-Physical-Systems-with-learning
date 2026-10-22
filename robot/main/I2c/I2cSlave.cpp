#include "I2cSlave.hpp"

I2cSlave::I2cSlave(uint16_t address) : address(address)
{
    setupSlave();
}

I2cSlave::~I2cSlave() = default;

void I2cSlave::setupSlave()
{
    devConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = sclSpeed,
        .scl_wait_us = 10000,
        .flags = {
            .disable_ack_check = 0,
        }};
}
