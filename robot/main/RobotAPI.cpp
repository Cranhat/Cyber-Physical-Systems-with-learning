#include "RobotAPI.hpp"

#include <cmath>
#include <cstring>
#include <functional>
#include <stdio.h>

namespace
{
    constexpr float pi = 3.1415F;
    constexpr float encoderPpr = 7.0F;
    constexpr float motorGearRatio = 150.0F;
    constexpr float quadratureEdges = 4.0F;
    constexpr float wheelRadiusInM = 0.016F;
    constexpr float wheelTrackInM = 0.16F;

    int distanceToEncoderTicks(float distanceInM)
    {
        const float countsPerRevolution = encoderPpr * motorGearRatio * quadratureEdges;
        const float wheelCircumferenceInM = 2.0F * pi * wheelRadiusInM;
        return static_cast<int>(0.9434F * std::lround(std::fabs(distanceInM) * countsPerRevolution / wheelCircumferenceInM));
    }

    int angleToEncoderTicks(float angleInDegrees)
    {
        const float angleInRadians = std::fabs(angleInDegrees) * pi / 180.0F;
        const float wheelDistanceInM = angleInRadians * wheelTrackInM / 2.0F;
        return distanceToEncoderTicks(wheelDistanceInM);
    }
}

RobotAPI::RobotAPI(Robot &robot) : robot(robot), dataExchanger(robot.dataExchanger)
{
    dataExchanger.appendToReceiving(std::bind(&RobotAPI::onRequest, this), receiveBuffer, sizeof(receiveBuffer));
}

RobotAPI::~RobotAPI() = default;

void RobotAPI::onRequest()
{
    // make sure that only one request is being processed at the time

    std::memcpy(&request, receiveBuffer, sizeof(request));

    switch (request.type)
    {
    case move:
        moveRobot();
        break;
    case rotate:
        rotateRobot();
        break;
    default:
        printf("Unknown robot request type: %d\n", static_cast<int>(request.type));
        break;
    }
}

void RobotAPI::moveRobot()
{
    int ticks = distanceToEncoderTicks(request.distanceInM);

    if (request.distanceInM < 0.0F)
    {
        ticks = -ticks;
    }

    robot.moveMotorsForEncoderTicksCount(ticks, slow);
}

void RobotAPI::rotateRobot()
{
    int ticks = angleToEncoderTicks(request.rotationAngle);

    if (request.rotationAngle < 0.0F)
    {
        ticks = -ticks;
    }

    robot.rotateMotorsForEncoderTicksCount(ticks, slow);
}
