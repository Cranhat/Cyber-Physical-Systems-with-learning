#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_attr.h>
#include <atomic>

/*
Class having two channels connected to encoders, counts pulses in interrupts.
*/
class Encoder
{
public:
    Encoder(const uint8_t encoderNumber, const uint16_t ppr, const gpio_num_t channelA, const gpio_num_t channelB, const float wheelRadius, const float tickCorrection);
    ~Encoder();

    void update();

    static void IRAM_ATTR gpio_isr_handler(void *arg);
    float getRevolutions(int64_t pulseCount) const;
    float getDistanceInMeters(int64_t pulseCount) const;
    float getCorrectedDistanceInMeters(int64_t pulseCount) const;
    float getVelocityInMpS();
    int64_t getPulseCount();
    void reset();

public:
    const uint8_t encoderNumber;
    const uint16_t ppr; // pulses per revolution
    const gpio_num_t channelA;
    const gpio_num_t channelB;
    const float wheelRadius;
    const float tickCorrection;

    std::atomic<int64_t> pulseCount{0};
    uint8_t previousState{0};
};
