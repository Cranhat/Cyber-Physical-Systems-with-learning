#include "Motor.hpp"

Motor::Motor(const gpio_num_t portNumberA, const gpio_num_t portNumberB, Encoder &encoder, Pwm &pwm)
    : portNumberA(portNumberA), portNumberB(portNumberB), encoder(encoder), pwm(pwm)
{
    pwm.addChannel(portNumberA, 0);
    pwm.addChannel(portNumberB, 0);
}

Motor::~Motor() = default;

void Motor::start(bool forward = true, PowerMode powerMode = slow)
{
    if (forward)
    {
        pwm[portNumberA].changeDuty(static_cast<uint8_t>(powerMode));
    }
    else
    {
        pwm[portNumberB].changeDuty(static_cast<uint8_t>(powerMode));
    }
}

void Motor::stop()
{
    pwm[portNumberA].changeDuty(0);
    pwm[portNumberB].changeDuty(0);
}

void Motor::setDuty(bool forward, uint8_t duty)
{
    if (forward)
    {
        pwm[portNumberA].changeDuty(duty);
        pwm[portNumberB].changeDuty(0);
    }
    else
    {
        pwm[portNumberA].changeDuty(0);
        pwm[portNumberB].changeDuty(duty);
    }
}

void Motor::resetEncoder()
{
    encoder.reset();
}

int64_t Motor::getEncoderTicks()
{
    return encoder.getPulseCount();
}
