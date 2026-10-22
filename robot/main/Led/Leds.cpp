#include "Leds.hpp"

Leds::Leds(Pwm &pwm) : pwm(pwm)
{
}

Leds::~Leds() = default;

void Leds::addLed(gpio_num_t portNumber)
{
    leds.insert({portNumber, Led(portNumber, pwm)});
}
