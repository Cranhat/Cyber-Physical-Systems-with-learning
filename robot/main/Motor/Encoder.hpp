#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_attr.h>
#include <atomic>
#include <chrono>

/*
Class having two channels connected to encoders. Store events to some buffer,
that will be further send to computer.
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
    float getVelocityInMpS();
    int64_t getPulseCount();
    void reset();

    int receiveData();

    uint8_t data[14];

public:
    const uint8_t encoderNumber;
    const uint16_t ppr; // pulses per revolution
    const gpio_num_t channelA;
    const gpio_num_t channelB;
    const float wheelRadius;
    const float tickCorrection;

    std::chrono::steady_clock::time_point previousTimestamp{
        std::chrono::steady_clock::now()};
    std::atomic<int64_t> pulseCount{0};
    std::atomic<int64_t> previousPulseCount{0};
    uint8_t previousState{0};
};
