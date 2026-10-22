#include "MotionEstimator.hpp"

#include "Config.hpp"
#include "freertos/task.h"
#include "esp_timer.h"
#include <cstring>

using namespace config;

namespace
{
    int writeFrame(uint8_t *frame, uint8_t type, float value, int64_t timeUs)
    {
        frame[0] = type;
        std::memcpy(frame + 1, &value, sizeof(value));
        std::memcpy(frame + 5, &timeUs, sizeof(timeUs));
        return MotionEstimator::frameSize;
    }
}

MotionEstimator::MotionEstimator(Accelerometer &accelerometer, Encoder &leftEncoder, Encoder &rightEncoder)
    : accelerometer(accelerometer),
      leftEncoder(leftEncoder),
      rightEncoder(rightEncoder),
      headingFilter(q_angle, q_gyroBias, r_angle, p_gyroBias),
      velocityFilter(q_velocity, q_accelBias, r_velocity, p_accelBias),
      newEstimate(xSemaphoreCreateBinary())
{
    xTaskCreate(task, "motion_estimator", estimationTaskStackSize, this, estimationTaskPriority, nullptr);
}

void MotionEstimator::setMotion(Motion newMotion)
{
    motion.store(newMotion);
    xSemaphoreTake(newEstimate, 0);
}

void MotionEstimator::waitForEstimate()
{
    xSemaphoreTake(newEstimate, portMAX_DELAY);
}

float MotionEstimator::getHeading() const
{
    return heading.load();
}

float MotionEstimator::getTurnRate() const
{
    return turnRate.load();
}

float MotionEstimator::getVelocity() const
{
    return velocity.load();
}

int MotionEstimator::sendVelocity()
{
    return writeFrame(velocityData, velocityPacketType, velocity.load(), estimateTimeUs.load());
}

int MotionEstimator::sendAngle()
{
    return writeFrame(angleData, anglePacketType, heading.load(), estimateTimeUs.load());
}

void MotionEstimator::task(void *arg)
{
    static_cast<MotionEstimator *>(arg)->run();
}

void MotionEstimator::run()
{
    int64_t previousLeftTicks = leftEncoder.getPulseCount();
    int64_t previousRightTicks = rightEncoder.getPulseCount();
    int64_t previousTimeUs = esp_timer_get_time();
    float encoderHeading = 0.0f;
    Motion previousMotion = Motion::still;
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&lastWakeTime, estimationDelay);

        float gyroZ = 0.0f;
        float accelForward = 0.0f;
        accelerometer.readMotion(gyroZ, accelForward);
        const float gyroRate = gyroDirection * gyroZ;

        // Real time since previous update, task can be late when other tasks run
        const int64_t now = esp_timer_get_time();
        const float dt = static_cast<float>(now - previousTimeUs) / 1000000.0f;
        previousTimeUs = now;

        const int64_t leftTicks = leftEncoder.getPulseCount();
        const int64_t rightTicks = rightEncoder.getPulseCount();
        const float leftDistance = leftEncoder.getCorrectedDistanceInMeters(leftTicks - previousLeftTicks);
        const float rightDistance = rightEncoder.getCorrectedDistanceInMeters(rightTicks - previousRightTicks);
        previousLeftTicks = leftTicks;
        previousRightTicks = rightTicks;

        const Motion currentMotion = motion.load();
        if (currentMotion == Motion::still)
        {
            // True turn rate is 0, so gyroscope reads only its bias.
            // Encoder heading follows estimate, so wheel slip of previous moves does not pull heading
            const float gyroBias = headingFilter.getSensorBias();
            headingFilter.setSensorBias(gyroBias + gyroBiasAveraging * (gyroRate - gyroBias));
            encoderHeading = headingFilter.getEstimate();
        }
        else if (previousMotion == Motion::still)
        {
            // New move: heading continues, velocity starts from 0
            headingFilter.reset(headingFilter.getEstimate());
            velocityFilter.reset();
        }
        previousMotion = currentMotion;

        // Clockwise: left wheel forward (+), right wheel backward (-)
        encoderHeading += (leftDistance - rightDistance) / wheelTrack;
        headingFilter.update(gyroRate, encoderHeading, dt);

        // While rotating velocity follows only encoders
        const float accel = currentMotion == Motion::rotating ? velocityFilter.getSensorBias() : accelForward;
        velocityFilter.update(accel, (leftDistance + rightDistance) / 2.0f / dt, dt);

        heading.store(headingFilter.getEstimate());
        turnRate.store(gyroRate - headingFilter.getSensorBias());
        velocity.store(velocityFilter.getEstimate());
        estimateTimeUs.store(now);
        xSemaphoreGive(newEstimate);
    }
}
