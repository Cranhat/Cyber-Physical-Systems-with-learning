#pragma once

#include "driver/gpio.h"
#include "Pwm/Pwm.hpp"

/*
Controls single LED with PWM. Allows duty and frequency changes;
*/

class Led
{
public:
    Led(gpio_num_t portNumber, Pwm &pwm);
    ~Led();
    void on();
    void blinkOn();
    void blinkOff();
    void off();
    void changeDuty(uint32_t duty);
    void changeFrequency(uint32_t frequency);

private:
    bool isBlinking = false;
    const gpio_num_t portNumber;
    Pwm &pwm;
    uint32_t blinkFrequency = 2;
};
