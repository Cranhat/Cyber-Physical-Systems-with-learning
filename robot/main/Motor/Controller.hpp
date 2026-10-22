#pragma once

#include <cstdint>
#include "Motor/Encoder.hpp"
#include "EKF/VelocityEkf.hpp"
#include "pid_ctrl.h"

/*
Closed-loop velocity controller for a single motor.
Uses the official espressif/pid_ctrl block and fuses
encoder + IMU acceleration through a VelocityEkf.

Usage:
  1. Call setTarget(velocity_mps) to set desired speed.
  2. Call tick(imuAccelMps2, dt) at a fixed rate — it reads
     the encoder, runs EKF, computes PID error and returns
     a PWM duty value in [0, 255].
*/
class Controller
{
public:
    Controller(Encoder &encoder);
    ~Controller();

    // Set desired velocity in m/s (positive = forward).
    void setTarget(float velocityMps);

    // Run one control cycle.
    //   imuAccelMps2 – forward acceleration from IMU (m/s²)
    //   dt           – elapsed time since last call (s)
    // Returns PWM duty [0, 255] to apply to the motor.
    uint8_t tick(float imuAccelMps2, float dt);
    uint8_t tick(float imuAccelMps2, float imuGyroZ, float dt, float encoderYaw = 0.0f);

    VelocityEkf &getEkf() { return ekf; }
    const VelocityEkf &getEkf() const { return ekf; }

    void reset();

private:
    Encoder &encoder;
    VelocityEkf ekf;
    pid_ctrl_block_handle_t pid;

    float targetVelocity{0.0f};
    int64_t lastPulseCount{0};
};
