#pragma once

#include "Socket.h"

#include <cstdint>

#define VELOCITY_TYPE 0x56
#define ANGLE_TYPE 0x41

//   VELOCITY 0x56: float velocity [m/s] along the direction of travel (from 0 each move), uint64 time_us
//   ANGLE    0x41: float heading [rad] since boot, clockwise positive,                    uint64 time_us
struct MotionFrame
{
    float velocity = 0.0F;
    uint64_t velocity_time_us = 0;
    float heading = 0.0F;
    uint64_t heading_time_us = 0;
};

class MotionData
{
 public:
    static constexpr int VALUE_SIZE = 4 + 8;
    static constexpr int PAIR_SIZE = VALUE_SIZE + 1 + VALUE_SIZE;  // rest of VELOCITY + whole ANGLE
    static constexpr float MAX_ABS_VELOCITY = 5.0F;                // m/s, the robot drives ~0.1 m/s
    static constexpr float MAX_ABS_HEADING = 100000.0F;            // rad, heading is kept since boot
    static constexpr uint64_t MAX_PAIR_TIME_DIFF_US = 100000;      // estimates come every 20 ms

    explicit MotionData(socket_t cfd);

    bool get_output() const { return output; }
    bool is_valid() const { return valid; }
    MotionFrame get_data() const { return data; }

 private:
    MotionFrame data{};
    bool output = false;
    bool valid = false;
};
