#include "LidarParser.h"
#include <chrono>

namespace
{
constexpr int LIDAR_PAYLOAD_SIZE = 1 + 2 + 2 + (POINT_PER_PACK * 3) + 2 + 2 + 1;

uint16_t readU16(const uint8_t* buf, int offset)
{
    return (uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8);
}
}

LidarData::LidarData(socket_t cfd, socket_t sfd, uint8_t packet_type)
    : cfd(cfd), sfd(sfd), packet_type(packet_type)
{
    output = LidarRecv();
}

void LidarData::printLidar()
{
    printf("===== LIDAR PACKET =====\n");

    printf("dataLength: %u\n", data.dataLength);
    printf("speed: %u\n", data.speed);
    printf("startAngle: %u\n", data.startAngle);

    printf("points:\n");

    for (int i = 0; i < POINT_PER_PACK; i++)
    {
        printf("  [%d] dist=%u conf=%u\n",
               i,
               data.point[i].distanceValue,
               data.point[i].confidence);
    }

    printf("endAngle: %u\n", data.endAngle);
    printf("timestamp: %u\n", data.timestamp);
    printf("crc: %u\n", data.crc8);
    
    printf("========================\n");
}

bool LidarData::LidarRecv()
{
    uint8_t buf[LIDAR_PAYLOAD_SIZE];
    int received = 0;

    while (received < (int)sizeof(buf))
    {
        int ret;
#ifdef _WIN32
        ret = recv(cfd, (char*)buf + received, sizeof(buf) - received, 0);

        if (ret < 0)
        {
            printf("Lidar recv error: %d\n", WSAGetLastError());
            closesocket(cfd);
            closesocket(sfd);

            WSACleanup();
#else
        ret = recv(cfd, buf + received, sizeof(buf) - received, 0);
    
        if (ret < 0)
        {
            perror("Lidar recv");
            close(cfd);
            close(sfd);
#endif
            return false;
    
        } else if (ret == 0) {
    
            return false;
        }

        received += ret;
    }

    writePacketDump(packet_type, buf, sizeof(buf));
    
    int offset = 0;

    data.dataLength = buf[offset];
    offset += sizeof(uint8_t);

    data.speed = readU16(buf, offset);
    offset += sizeof(uint16_t);

    data.startAngle = readU16(buf, offset);
    offset += sizeof(uint16_t);

    for (int i = 0; i < POINT_PER_PACK; i++)
    {
        data.point[i].distanceValue = readU16(buf, offset);
        offset += sizeof(uint16_t);

        data.point[i].confidence = buf[offset];
        offset += sizeof(uint8_t);
    }

    data.endAngle = readU16(buf, offset);
    offset += sizeof(uint16_t);

    data.timestamp = readU16(buf, offset);
    offset += sizeof(uint16_t);
    
    data.crc8 = buf[offset];
    
    static auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto now = std::chrono::steady_clock::now();

    if (now - last_print >= std::chrono::seconds(1))
    {
        printLidar();
        last_print = now;
    }

    return true;
}

bool LidarData::get_output()
{
    return output;
}
