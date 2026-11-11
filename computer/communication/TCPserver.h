#pragma once

#include "LidarParser.h"
#include "MotionParser.h"
#include "StatusParser.h"

#include <functional>
#include <mutex>

#define BACKLOG 10

#define LIDAR_TYPE 0x54
#define RESPONSE_TYPE 0x52
#define STATE_TYPE 0x53

class TCPserver
{
 private:
    socket_t cfd;
    socket_t sfd;
    const char* PORT;
    std::string peerAddr;
    std::mutex sockMutex;  // cfd/sfd are also used by the sending thread

    // The robot starts every loop with its RESPONSE/STATE packets, then lidar bytes, then
    // VELOCITY + ANGLE. 0x52/0x53 are only taken right there (after ANGLE, another
    // RESPONSE/STATE or at connection start), so the same bytes inside lidar data are skipped.
    bool statusSlot = false;

 public:
    TCPserver(const char* PORT);
    ~TCPserver();

    std::function<void(const LiData&)> onLidar;
    std::function<void(const MotionFrame&)> onMotion;  // VELOCITY + ANGLE pair
    std::function<void(const ResponseFrame&)> onResponse;
    std::function<void(RobotState)> onState;

    bool LidarHandl();
    bool MotionHandl();
    bool ResponseHandl();
    bool StateHandl();

    bool connectSock();
    bool receiveData();

    // PC -> robot
    bool sendData(const uint8_t* data, size_t size);
    void closeSock();
    void shutdownSock();
    std::string peer();
};

