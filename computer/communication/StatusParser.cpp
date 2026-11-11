#include "StatusParser.h"
#include "SocketPeek.h"

static uint32_t readU32(const uint8_t* buf, int offset)
{
    return (uint32_t)buf[offset] | ((uint32_t)buf[offset + 1] << 8) | ((uint32_t)buf[offset + 2] << 16) |
           ((uint32_t)buf[offset + 3] << 24);
}

const char* toString(RobotState state)
{
    switch (state)
    {
    case RobotState::Idle:
        return "IDLE";
    case RobotState::Calibrating:
        return "CALIBRATING";
    case RobotState::Ready:
        return "READY";
    case RobotState::Processing:
        return "PROCESSING";
    case RobotState::Failure:
        return "FAILURE";
    }
    return "UNKNOWN";
}

ResponseData::ResponseData(socket_t cfd)
{
    uint8_t buf[12];

    const PeekResult peek = peekBytes(cfd, buf, sizeof(buf));
    output = peek != PeekResult::Closed;  // false: connection broken
    if (peek != PeekResult::Ok)
    {
        return;  // stalled: not a whole packet, valid stays false
    }
    data.id = readU32(buf, 0);
    data.status = readU32(buf, 4);
    data.time_ms = readU32(buf, 8);

    // ids start at 1 (see RobotLink); status is 0 or 1; no request takes longer than ~1 h
    valid = data.id != 0 && data.status <= 1 && data.time_ms < 3600u * 1000u;

    if (valid)
    {
        output = takeBytes(cfd, buf, sizeof(buf));
    }
}

StateData::StateData(socket_t cfd)
{
    uint8_t buf[4];

    const PeekResult peek = peekBytes(cfd, buf, sizeof(buf));
    output = peek != PeekResult::Closed;  // false: connection broken
    if (peek != PeekResult::Ok)
    {
        return;  // stalled: not a whole packet, valid stays false
    }
    const uint32_t value = readU32(buf, 0);
    valid = value <= static_cast<uint32_t>(RobotState::Failure);

    if (valid)
    {
        data = static_cast<RobotState>(value);
        output = takeBytes(cfd, buf, sizeof(buf));
    }
}
