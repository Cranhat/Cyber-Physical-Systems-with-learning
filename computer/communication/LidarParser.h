#pragma once

#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>

#include "Socket.h"

#define POINT_PER_PACK 12

struct LidarPoints
{
    uint16_t distanceValue;
    uint8_t confidence;
};

struct LiData
{
    int8_t      dataLength;
    uint16_t    speed;
    uint16_t    startAngle;
    LidarPoints point[POINT_PER_PACK];
    uint16_t    endAngle;
    uint16_t    timestamp;
    uint8_t     crc8;
};

class LidarData
{
 private:
    LiData data;

    socket_t cfd, sfd;
    uint8_t packet_type;
    bool output;
    bool valid{false};

    static constexpr int LIDAR_PAYLOAD_SIZE = 1 + 2 + 2 + (POINT_PER_PACK * 3) + 2 + 2 + 1;

    static uint16_t readU16(const uint8_t* buf, int offset);
    // LD06 CRC8 (poly 0x4D) over header 0x54 + payload without the crc byte
    static uint8_t lidarCrc8(const uint8_t* buf, int size);
 public:
    LidarData() = delete;
    LidarData(socket_t cfd, socket_t sfd, uint8_t packet_type);

    void printLidar();
    bool LidarRecv();
    bool get_output();
    bool is_valid();
    LiData get_data();
};
