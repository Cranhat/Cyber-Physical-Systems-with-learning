#pragma once

#include "Socket.h"
#include <cstdint>

// RESPONSE packet (0x52, 1 + 12 B)
struct ResponseFrame
{
    uint32_t id;
    uint32_t status;   // 0 = succeeded, 1 = failed
    uint32_t time_ms;
};

// STATE packet (0x53, 1 + 4 B): robot state, on every change and every 10 s.
enum class RobotState : uint32_t
{
    Idle = 0,
    Calibrating = 1,
    Ready = 2,
    Processing = 3,
    Failure = 4,
};

const char* toString(RobotState state);

class ResponseData
{
 public:
    explicit ResponseData(socket_t cfd);
    bool get_output() const { return output; }
    bool is_valid() const { return valid; }
    ResponseFrame get_data() const { return data; }

 private:
    ResponseFrame data{};
    bool output = false;
    bool valid = false;
};

class StateData
{
 public:
    explicit StateData(socket_t cfd);
    bool get_output() const { return output; }
    bool is_valid() const { return valid; }
    RobotState get_data() const { return data; }

 private:
    RobotState data = RobotState::Idle;
    bool output = false;
    bool valid = false;
};
