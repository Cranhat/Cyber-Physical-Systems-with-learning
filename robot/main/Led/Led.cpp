#include "Led/Led.hpp"

#include <thread>
#include <chrono>

#include "freertos/FreeRTOS.h"

Led::Led(gpio_num_t portNumber, Pwm &pwm) : portNumber(portNumber), pwm(pwm)
{
    pwm.addChannel(portNumber, 0);
}

Led::~Led()
{
    off();
}

void Led::on()
{
    changeDuty(255);
}

void Led::off()
{
    changeDuty(0);
}

void Led::blinkOn()
{
    std::thread th([&]()
                   {    isBlinking = true;
    while(isBlinking){
        changeDuty(255);
        vTaskDelay(1000/portTICK_PERIOD_MS); 
        changeDuty(0);
        vTaskDelay(1000/portTICK_PERIOD_MS); 
    } });
    th.detach();
}

void Led::blinkOff()
{
    isBlinking = true;
}

void Led::changeDuty(uint32_t duty)
{
    pwm[portNumber].changeDuty(duty);
}

void Led::changeFrequency(uint32_t frequency)
{
    pwm[portNumber].changeFrequency(frequency);
}
