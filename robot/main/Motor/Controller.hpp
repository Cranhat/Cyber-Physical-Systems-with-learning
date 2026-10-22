#pragma once

#include <cstdint>
#include "pid_ctrl.h"

/*
Closed-loop speed controller.
Feedforward duty from target speed plus correction from the official
espressif/pid_ctrl block, with speed estimated by KalmanFilter in MotorManager.
Feedforward starts both motors with a sensible duty at once,
so PID only corrects small differences.

Usage:
  1. Call setTarget(velocity_mps) to set desired speed.
  2. Call tick(measuredVelocity) at a fixed rate — it computes
     feedforward + PID and returns a PWM duty value in [0, maxOutput].
*/

class Controller
{
public:
    Controller(float maxOutput, float velocityToDuty);
    ~Controller();

    void setTarget(float velocityMps);

    uint8_t tick(float measuredVelocity);

    void reset();

private:
    const float maxOutput;
    const float velocityToDuty;
    pid_ctrl_block_handle_t pid;

    float targetVelocity{0.0f};

    void createPid();
};
