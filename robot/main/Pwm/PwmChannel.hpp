#pragma once

#include <cstdint>
#include "driver/ledc.h"
#include "driver/gpio.h"

class PwmChannel
{
public:
    PwmChannel(gpio_num_t portNumber, uint8_t duty);
    ~PwmChannel();

    void changeDuty(uint8_t duty);
    void changeFrequency(uint32_t frequency);

    ledc_channel_t reserveNextChannel();

private:
    ledc_channel_t channel;
    uint8_t duty;
    uint32_t frequency;
};
