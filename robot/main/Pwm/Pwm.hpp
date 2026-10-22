#pragma once

#include "driver/ledc.h"
#include "driver/gpio.h"
#include <map>
#include "PwmChannel.hpp"

/*
Controls pwm and allows to add new channels.
*/
class Pwm
{
public:
    Pwm(uint32_t pwmFrequency);
    ~Pwm();

    void addChannel(gpio_num_t portNumber, uint8_t duty);

    PwmChannel &operator[](gpio_num_t portNumber)
    {
        return pwmChannels.at(portNumber);
    }

private:
    uint32_t pwmFrequency;
    std::map<gpio_num_t, PwmChannel> pwmChannels;
};
