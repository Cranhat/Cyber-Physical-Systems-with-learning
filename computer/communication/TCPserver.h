#pragma once

#include "LidarParser.h"
#include "AccelParser.h"
#include "EncoderParser.h"
 
#define BACKLOG 10

#define LIDAR_TYPE 0x54
#define ACCEL_TYPE 0x67
#define ENCODER_TYPE 0x68

class TCPserver
{
 private:
    socket_t cfd;
    socket_t sfd;
    const char* PORT;
    
 public:
    TCPserver(const char* PORT);
    ~TCPserver();

    bool AccelHandl();
    bool LidarHandl();
    bool EncoderHandl();
 
    bool connectSock();
    bool receiveData();
};

