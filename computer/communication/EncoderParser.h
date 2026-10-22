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

struct EncoderFrame
{
    uint8_t encoder;
    float meters;
    uint64_t timestamp_delta_ns;
};

class EncoderData
{
 private:
    EncoderFrame data;

    socket_t cfd, sfd;
    uint8_t packet_type;
    bool output;
 public:
    EncoderData() = delete;
    EncoderData(socket_t cfd, socket_t sfd, uint8_t packet_type);

    void printEncoder();
    bool EncoderRecv();
    bool get_output();
    EncoderFrame get_data();
};
