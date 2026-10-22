#include "Controller.hpp"

namespace
{
    // PID gains — tune these for your robot
    constexpr float Kp          = 80.0f;
    constexpr float Ki          = 10.0f;
    constexpr float Kd          = 2.0f;

    // Output maps directly to PWM duty [0, 255]
    constexpr float maxOutput   = 255.0f;
    constexpr float minOutput   = 0.0f;

    // Integral wind-up clamp
    constexpr float maxIntegral = 100.0f;
    constexpr float minIntegral = -100.0f;
}

Controller::Controller(Encoder &encoder) : encoder(encoder)
{
    pid_ctrl_config_t config = {
        .init_param = {
            .kp           = Kp,
            .ki           = Ki,
            .kd           = Kd,
            .max_output   = maxOutput,
            .min_output   = minOutput,
            .max_integral = maxIntegral,
            .min_integral = minIntegral,
            .cal_type     = PID_CAL_TYPE_POSITIONAL,
        }
    };

    pid_new_control_block(&config, &pid);
    lastPulseCount = encoder.getPulseCount();
}

Controller::~Controller()
{
    pid_del_control_block(pid);
}

void Controller::setTarget(float velocityMps)
{
    targetVelocity = velocityMps;
}

uint8_t Controller::tick(float imuAccelMps2, float dt)
{
    return tick(imuAccelMps2, 0.0f, dt, 0.0f);
}

uint8_t Controller::tick(float imuAccelMps2, float imuGyroZ, float dt, float encoderYaw)
{
    // Encoder velocity: Δpulses / dt → m/s
    const int64_t currentPulseCount = encoder.getPulseCount();
    const int64_t deltaPulses = currentPulseCount - lastPulseCount;
    lastPulseCount = currentPulseCount;

    const float encoderVel = (dt > 0.0f)
        ? encoder.getDistanceInMeters(deltaPulses) / dt
        : 0.0f;

    // Fuse encoder + IMU (accel and gyro) through 4-state EKF
    const float fusedVel = ekf.update(imuAccelMps2, imuGyroZ, encoderVel, encoderYaw, dt);

    // PID: error = target - measured
    const float error = targetVelocity - fusedVel;
    float output = 0.0f;
    pid_compute(pid, error, &output);

    return static_cast<uint8_t>(output);
}

void Controller::reset()
{
    ekf.reset();
    lastPulseCount = encoder.getPulseCount();
    targetVelocity = 0.0f;
    // Re-create the pid block to clear its internal state
    pid_del_control_block(pid);
    pid_ctrl_config_t config = {
        .init_param = {
            .kp           = Kp,
            .ki           = Ki,
            .kd           = Kd,
            .max_output   = maxOutput,
            .min_output   = minOutput,
            .max_integral = maxIntegral,
            .min_integral = minIntegral,
            .cal_type     = PID_CAL_TYPE_POSITIONAL,
        }
    };
    pid_new_control_block(&config, &pid);
}
