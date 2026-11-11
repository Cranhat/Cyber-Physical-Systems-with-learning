#include "LidarParser.h"
#include "SocketPeek.h"
#include <chrono>

uint16_t LidarData::readU16(const uint8_t* buf, int offset)
{
    return (uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8);
}

uint8_t LidarData::lidarCrc8(const uint8_t* buf, int size)
{
    uint8_t crc = 0;

    for (int i = -1; i < size; i++)
    {
        crc ^= (i < 0) ? (uint8_t)0x54 : buf[i];

        for (int bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
        }
    }

    return crc;
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

    // look first, take the bytes from the socket only if the packet is valid
    const PeekResult peek = peekBytes(cfd, buf, sizeof(buf));
    if (peek != PeekResult::Ok)
    {
        return peek == PeekResult::Stalled;  // stalled: not a whole frame, valid stays false
    }
    
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

    valid = (uint8_t)data.dataLength == 0x2C && data.crc8 == lidarCrc8(buf, LIDAR_PAYLOAD_SIZE - 1);

    if (!valid)
    {
        return true;  // not a packet: the bytes stay in the socket
    }

    // valid packet -> take it from the socket
    int received = 0;

    while (received < (int)sizeof(buf))
    {
        int ret;
#ifdef _WIN32
        ret = recv(cfd, (char*)buf + received, sizeof(buf) - received, 0);
#else
        ret = recv(cfd, buf + received, sizeof(buf) - received, 0);
#endif

        if (ret < 0)
        {
#ifdef _WIN32
            printf("Lidar recv error: %d\n", WSAGetLastError());
#else
            perror("Lidar recv");
#endif
            return false;
        }
        else if (ret == 0)
        {
            return false;
        }

        received += ret;
    }

    return true;
}

bool LidarData::get_output()
{
    return output;
}

bool LidarData::is_valid()
{
    return valid;
}

LiData LidarData::get_data()
{
    return data;
}
