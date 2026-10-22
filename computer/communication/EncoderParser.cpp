#include "EncoderParser.h"
#include <chrono>

namespace
{
constexpr int ENCODER_PAYLOAD_SIZE = 1 + 4 + 8;

uint64_t readU64(const uint8_t* buf, int offset)
{
    uint64_t value = 0;

    for (int i = 0; i < 8; i++)
    {
        value |= ((uint64_t)buf[offset + i]) << (8 * i);
    }

    return value;
}
}

EncoderData::EncoderData(socket_t cfd, socket_t sfd, uint8_t packet_type)
    : cfd(cfd), sfd(sfd), packet_type(packet_type)
{
    output = EncoderRecv();
}

void EncoderData::printEncoder()
{
    printf("===== ENCODER DATA =====\n");
    printf("encoder: %u\n", data.encoder);
    printf("meters: %f\n", data.meters);
    printf("timestamp_delta_ns: %llu\n", (unsigned long long)data.timestamp_delta_ns);
    printf("========================\n");
}

bool EncoderData::EncoderRecv()
{
    uint8_t buf[ENCODER_PAYLOAD_SIZE];
    int received = 0;

    while (received < (int)sizeof(buf))
    {
        int ret;
#ifdef _WIN32
        ret = recv(cfd, (char*)buf + received, sizeof(buf) - received, 0);

        if (ret < 0)
        {
            printf("Encoder recv error: %d\n", WSAGetLastError());
            closesocket(cfd);
            closesocket(sfd);

            WSACleanup();
#else
        ret = recv(cfd, buf + received, sizeof(buf) - received, 0);

        if (ret < 0)
        {
            perror("Encoder recv");
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

    data.encoder = buf[0];

    uint32_t meters_bits =
        ((uint32_t)buf[1]) |
        ((uint32_t)buf[2] << 8) |
        ((uint32_t)buf[3] << 16) |
        ((uint32_t)buf[4] << 24);

    std::memcpy(&data.meters, &meters_bits, sizeof(data.meters));

    if (data.encoder == 2)
    {
        data.meters = -data.meters;
    }

    data.timestamp_delta_ns = readU64(buf, 5);

    static auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto now = std::chrono::steady_clock::now();

    if (now - last_print >= std::chrono::seconds(1))
    {
        printEncoder();
        last_print = now;
    }

    return true;
}

bool EncoderData::get_output()
{
    return output;
}

EncoderFrame EncoderData::get_data()
{
    return data;
}
