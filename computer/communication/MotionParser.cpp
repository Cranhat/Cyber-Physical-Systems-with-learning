#include "MotionParser.h"
#include "SocketPeek.h"

#include <cmath>
#include <cstring>

static float readF32(const uint8_t* buf, int offset)
{
    const uint32_t bits = (uint32_t)buf[offset] | ((uint32_t)buf[offset + 1] << 8) |
                          ((uint32_t)buf[offset + 2] << 16) | ((uint32_t)buf[offset + 3] << 24);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint64_t readU64(const uint8_t* buf, int offset)
{
    uint64_t value = 0;

    for (int i = 0; i < 8; i++)
    {
        value |= ((uint64_t)buf[offset + i]) << (8 * i);
    }

    return value;
}

MotionData::MotionData(socket_t cfd)
{
    uint8_t buf[PAIR_SIZE];

    const PeekResult peek = peekBytes(cfd, buf, sizeof(buf));
    output = peek != PeekResult::Closed;  // false: connection broken
    if (peek != PeekResult::Ok)
    {
        return;  // stalled: not a whole packet, valid stays false
    }

    data.velocity = readF32(buf, 0);
    data.velocity_time_us = readU64(buf, 4);
    data.heading = readF32(buf, VALUE_SIZE + 1);
    data.heading_time_us = readU64(buf, VALUE_SIZE + 1 + 4);

    const uint64_t timeDiff = data.velocity_time_us > data.heading_time_us
                                  ? data.velocity_time_us - data.heading_time_us
                                  : data.heading_time_us - data.velocity_time_us;

    valid = buf[VALUE_SIZE] == ANGLE_TYPE &&
            std::isfinite(data.velocity) && std::fabs(data.velocity) <= MAX_ABS_VELOCITY &&
            std::isfinite(data.heading) && std::fabs(data.heading) <= MAX_ABS_HEADING &&
            data.velocity_time_us > 0 && data.heading_time_us > 0 && timeDiff <= MAX_PAIR_TIME_DIFF_US;

    if (valid)
    {
        output = takeBytes(cfd, buf, sizeof(buf));
    }
}
