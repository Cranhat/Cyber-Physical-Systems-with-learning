#include "Controller.hpp"

#include <algorithm>

namespace
{
    // PID gains — tune these for your robot
    constexpr float Kp          = 80.0f;
    constexpr float Ki          = 10.0f;
    constexpr float Kd          = 2.0f;
}

Controller::Controller(float maxOutput, float velocityToDuty) : maxOutput(maxOutput), velocityToDuty(velocityToDuty)
{
    createPid();
}

Controller::~Controller()
{
    pid_del_control_block(pid);
}

void Controller::createPid()
{
    pid_ctrl_config_t config = {
        .init_param = {
            .kp           = Kp,
            .ki           = Ki,
            .kd           = Kd,
            .max_output   = maxOutput,
            .min_output   = -maxOutput,
            .max_integral = maxOutput / Ki,
            .min_integral = -maxOutput / Ki,
            .cal_type     = PID_CAL_TYPE_POSITIONAL,
        }
    };

    pid_new_control_block(&config, &pid);
}

void Controller::setTarget(float velocityMps)
{
    targetVelocity = velocityMps;
}

uint8_t Controller::tick(float measuredVelocity)
{
    const float error = targetVelocity - measuredVelocity;
    float correction = 0.0f;
    pid_compute(pid, error, &correction);

    const float output = targetVelocity * velocityToDuty + correction;
    return static_cast<uint8_t>(std::clamp(output, 0.0f, maxOutput));
}

void Controller::reset()
{
    targetVelocity = 0.0f;
    pid_del_control_block(pid);
    createPid();
}
