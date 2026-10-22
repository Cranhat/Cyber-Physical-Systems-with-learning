#pragma once

#include <driver/i2c_master.h>
#include <vector>
#include "I2cSlave.hpp"
#include <memory>

#define DEFAULT_TIMEOUT 2000

class I2c
{
public:
    I2c(uint16_t address, i2c_port_num_t i2cPort, gpio_num_t sdaPortNumber, gpio_num_t sclPortNumber);
    ~I2c();

    std::shared_ptr<I2cSlave> createSlave(uint8_t reg);
    void transmit(std::shared_ptr<I2cSlave> slave, uint8_t reg, uint8_t *cmd, uint8_t writeSize);
    void transmitReceive(std::shared_ptr<I2cSlave> slave, uint8_t reg, uint8_t *buffer, uint8_t writeSize);

private:
    const uint16_t address;
    const i2c_port_num_t i2cPort;
    const gpio_num_t sdaPortNumber;
    const gpio_num_t sclPortNumber;
    std::vector<std::shared_ptr<I2cSlave>> slaves;
    esp_err_t esp_ret;
    i2c_master_bus_handle_t bus;

    void setUpMaster();
};
