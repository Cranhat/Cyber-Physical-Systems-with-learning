#pragma once

#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include "PacketDump.h"

#ifdef _WIN32
    #define _WINSOCK_DEPRECATED_NO_WARNINGS

    #include <winsock2.h>
    #include <ws2tcpip.h>

    #pragma comment(lib, "ws2_32.lib")

    using socket_t = SOCKET;

#else
    #include <unistd.h>
    #include <errno.h>

    #include <netdb.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>

    using socket_t = int;

#endif

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
 public:
    LidarData() = delete;
    LidarData(socket_t cfd, socket_t sfd, uint8_t packet_type);

    void printLidar();
    bool LidarRecv();
    bool get_output();
};
