#include "Encoder.hpp"
#include <cmath>
#include <driver/gpio.h>
#include <bit>

#define PI 3.14

Encoder::Encoder(const uint8_t encoderNumber, const uint16_t ppr, const gpio_num_t channelA, const gpio_num_t channelB, const float wheelRadius, const float tickCorrection)
    : encoderNumber(encoderNumber), ppr(ppr), channelA(channelA), channelB(channelB), wheelRadius(wheelRadius), tickCorrection(tickCorrection)
{
    gpio_config_t io_conf{};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << channelA) | (1ULL << channelB);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_isr_handler_add(channelA, Encoder::gpio_isr_handler, this);
    gpio_isr_handler_add(channelB, Encoder::gpio_isr_handler, this);

    previousState = (gpio_get_level(channelA) << 1) | gpio_get_level(channelB);
    data[0] = 0x68;
    data[1] = encoderNumber;
}

Encoder::~Encoder()
{
    gpio_isr_handler_remove(channelA);
    gpio_isr_handler_remove(channelB);
}

void IRAM_ATTR Encoder::gpio_isr_handler(void *arg)
{
    Encoder *encoder = static_cast<Encoder *>(arg);
    encoder->update();
}

void IRAM_ATTR Encoder::update()
{
    uint8_t currentState = (gpio_get_level(channelA) << 1) | gpio_get_level(channelB);

    if (currentState == previousState)
    {
        return;
    }

    uint8_t state = (previousState << 2) | currentState;
    switch (state)
    {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        pulseCount.fetch_add(1);
        break;

    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
        pulseCount.fetch_sub(1);
        break;
    }

    previousState = currentState;
}

float Encoder::getRevolutions(int64_t pulseCount) const
{
    const float countsPerRevolution = static_cast<float>(ppr) * 150.0F * 4.0F;
    return static_cast<float>(pulseCount) / countsPerRevolution;
}

float Encoder::getDistanceInMeters(int64_t pulseCount) const
{
    const float circumference = 2.0 * static_cast<float>(PI) * wheelRadius;
    return getRevolutions(pulseCount) * circumference;
}

void Encoder::reset()
{
    pulseCount.store(0);
}

int64_t Encoder::getPulseCount()
{
    return pulseCount.load();
}

int Encoder::receiveData()
{
    auto currentTimestamp = std::chrono::steady_clock::now();
    const std::chrono::duration elapsed{currentTimestamp - previousTimestamp};
    previousTimestamp = currentTimestamp;

    int64_t currentPulseCount = pulseCount.load();
    int64_t sentPulseCount = currentPulseCount - previousPulseCount.load();
    previousPulseCount.store(currentPulseCount);
    int32_t meters = std::bit_cast<int32_t>(getDistanceInMeters(sentPulseCount));
    data[2] = meters & 0xFF;
    data[3] = (meters & 0xFF00) >> 8;
    data[4] = (meters & 0xFF0000) >> 16;
    data[5] = (meters & 0xFF000000) >> 24;

    uint64_t nano = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
    for (int i = 6; i < 6 + 8; i++)
    {
        data[i] = nano & 0xFF;
        nano >>= 8;
    }
    return 14;
}
