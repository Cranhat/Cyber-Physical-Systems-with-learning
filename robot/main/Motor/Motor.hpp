#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include "Motor/Encoder.hpp"
#include "Pwm/Pwm.hpp"

enum Direction
{
    forward,
    reverse
};

enum PowerMode // represented in x/255 %
{
    ultraSlow = 100,
    slow = 150,
    normal = 180,
    mediumFast = 200,
    fast = 220,
    overdrive = 255
};

/*
Class providing usage of single motor via PWM. Should calculate
ticks needed based on wheel diameter and allow to move by certain distance.

*/
class Motor
{
public:
    Motor(const gpio_num_t portNumberA, const gpio_num_t portNumberB, Encoder &encoder, Pwm &pwm);
    ~Motor();

    void start(bool forward, PowerMode powerMode);
    void setDuty(bool forward, uint8_t duty);
    void stop();
    void resetEncoder();
    int64_t getEncoderTicks();

    const gpio_num_t portNumberA;
    const gpio_num_t portNumberB;
    Encoder &encoder;
    Pwm &pwm;
};
