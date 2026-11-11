#pragma once

#include <algorithm>

/*
PID controller with constant gains and fixed loop period.
Output and integral part are both limited to [-maxOutput, maxOutput].
Create a new one for every move, so state from previous move is not carried over.
*/
class Pid
{
public:
    Pid(float kp, float ki, float kd, float maxOutput, float dt)
        : kp(kp), ki(ki), kd(kd), maxOutput(maxOutput), dt(dt)
    {
    }

    float update(float error)
    {
        // First sample has no previous error, derivative would jump
        const float derivative = first ? 0.0f : (error - previousError) / dt;
        first = false;
        previousError = error;

        if (ki != 0.0f)
        {
            integral = std::clamp(integral + error * dt, -maxOutput / ki, maxOutput / ki);
        }

        return std::clamp(kp * error + ki * integral + kd * derivative, -maxOutput, maxOutput);
    }

private:
    const float kp;
    const float ki;
    const float kd;
    const float maxOutput;
    const float dt;

    float integral{0.0f};
    float previousError{0.0f};
    bool first{true};
};
