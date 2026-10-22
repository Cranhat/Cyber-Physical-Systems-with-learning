#pragma once

#include "Robot/Robot.hpp"
#include <atomic>
#include <cstdint>

enum RequestType : uint32_t
{
    move,
    rotate
};

struct Request
{
    RequestType type;    // 4 bytes
    float distanceInM;   // 4 bytes
    float rotationAngle; // 4 bytes
};

/*
Class used to provide driving interface via computer connection.
It takes Robot and its reference to Data Exchanger.

This might not be true. We'll find out how to do this later.
*/
class RobotAPI
{
public:
    RobotAPI(Robot &robot);
    ~RobotAPI();

    void onRequest();
    void moveRobot();
    void rotateRobot();

private:
    Robot &robot;
    DataExchanger &dataExchanger;
    Request request{};
    uint8_t receiveBuffer[sizeof(Request)]{};
};
