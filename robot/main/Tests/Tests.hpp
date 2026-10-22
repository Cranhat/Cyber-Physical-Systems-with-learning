#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <stdio.h>

#include "Config.hpp"
#include "Accelerometer/Accelerometer.hpp"
#include "Kalman/MotionEstimator.hpp"
#include "Motor/Encoder.hpp"
#include "Motor/Motor.hpp"
#include "Motor/MotorManager.hpp"

/*
Tests run from app_main instead of normal operation.
*/
namespace tests
{
    // Wheels lifted: each motor runs forward, then reverse, and encoder ticks are printed
    inline void motors(Motor &motorLeft, Motor &motorRight)
    {
        printf("\n---motor test---\n");
        Motor *testMotors[] = {&motorLeft, &motorRight};
        const char *names[] = {"left", "right"};
        for (int i = 0; i < 2; i++)
        {
            for (bool forward : {true, false})
            {
                const int64_t start = testMotors[i]->getEncoderTicks();
                testMotors[i]->setDuty(forward, 150);
                vTaskDelay(pdMS_TO_TICKS(1000));
                testMotors[i]->stop();
                vTaskDelay(pdMS_TO_TICKS(500));
                printf("%s %s: %lld ticks\n", names[i], forward ? "forward" : "reverse",
                       static_cast<long long>(testMotors[i]->getEncoderTicks() - start));
            }
        }
        printf("---motor test done---\n");
    }

    // Motors stay off, robot is moved by hand and sensor directions are printed, never returns.
    // Rotate robot clockwise by hand (seen from above): both headings must grow positive.
    // Push robot forward by hand: both wheel distances and velocity must be positive.
    inline void sensors(Accelerometer &accelerometer, Encoder &encoderLeft, Encoder &encoderRight)
    {
        printf("\n---sensor test---\n");
        constexpr float dt = 0.020f;
        constexpr float radToDeg = 180.0f / 3.14159265f;
        const int64_t leftStart = encoderLeft.getPulseCount();
        const int64_t rightStart = encoderRight.getPulseCount();
        float gyroHeading = 0.0f;
        float velocity = 0.0f;
        int sample = 0;
        TickType_t lastWakeTime = xTaskGetTickCount();

        while (true)
        {
            float gyroZ = 0.0f;
            float accelForward = 0.0f;
            accelerometer.readMotion(gyroZ, accelForward);
            gyroHeading += -1.0f * gyroZ * dt; // same gyroDirection as MotionEstimator
            velocity += accelForward * dt;

            if (++sample % 10 == 0)
            {
                const float left = encoderLeft.getDistanceInMeters(encoderLeft.getPulseCount() - leftStart);
                const float right = encoderRight.getDistanceInMeters(encoderRight.getPulseCount() - rightStart);
                const float encoderHeading = (left - right) / config::wheelTrack;
                printf("gyro heading %7.1f deg | encoder heading %7.1f deg | left %6.3f m | right %6.3f m | accel %6.2f m/s2 | accel velocity %6.2f m/s | right pins A=%d B=%d ticks=%lld\n",
                       gyroHeading * radToDeg, encoderHeading * radToDeg, left, right, accelForward, velocity,
                       gpio_get_level(encoderRight.channelA), gpio_get_level(encoderRight.channelB),
                       static_cast<long long>(encoderRight.getPulseCount()));
            }
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
        }
    }

    // Robot on the floor: rotates 45 deg clockwise, then 45 deg counter-clockwise, every sample is logged
    inline void rotation(MotorManager &motorManager)
    {
        printf("\n---rotation test---\n");
        motorManager.logRotation = true;
        printf("--- 45 deg clockwise ---\n");
        motorManager.rotateForAngle(45, normal);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        printf("--- 45 deg counter-clockwise ---\n");
        motorManager.rotateForAngle(-45, normal);
        printf("---rotation test done---\n");
    }

    // Robot on the floor: drives 0.5 m, turns back, drives 0.5 m and turns back again
    inline void driving(MotorManager &motorManager)
    {
        motorManager.moveForDistance(0.5, normal);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        motorManager.rotateForAngle(180, normal);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        motorManager.moveForDistance(0.5, normal);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        motorManager.rotateForAngle(180, normal);
        printf("---driving test done---\n");
    }
}
