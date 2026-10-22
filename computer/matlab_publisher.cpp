#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

namespace
{
constexpr uint8_t LIDAR_TYPE = 0x54;
constexpr uint8_t ACCEL_TYPE = 0x67;
constexpr uint8_t ENCODER_TYPE = 0x68;

constexpr size_t LIDAR_PACKET_SIZE = 47;
constexpr size_t ACCEL_PACKET_SIZE = 21;
constexpr size_t ENCODER_PACKET_SIZE = 14;
constexpr int POINT_PER_PACK = 12;

constexpr size_t GYRO_Z_OFFSET = 5;
constexpr size_t ACCEL_X_OFFSET = 7;
constexpr size_t ACCEL_Y_OFFSET = 9;
constexpr size_t ACCEL_DT_OFFSET = 13;

constexpr size_t ENCODER_ID_OFFSET = 1;
constexpr size_t ENCODER_DISTANCE_OFFSET = 2;
constexpr size_t ENCODER_DT_OFFSET = 6;
constexpr uint64_t MAX_ACCEL_DT_NS = 200000000;
constexpr int MAX_GYRO_RAW = 26214; // 200 dps on a +/-250 dps scale.
constexpr int MAX_ACCEL_AXIS_RAW = 30000;
constexpr int64_t MIN_ACCEL_MAGNITUDE_SQ = 4000000;
constexpr int64_t MAX_ACCEL_MAGNITUDE_SQ = 900000000;

uint16_t readU16(const std::vector<uint8_t>& data, size_t offset)
{
    return (uint16_t)data[offset] | ((uint16_t)data[offset + 1] << 8);
}

int16_t readI16(const std::vector<uint8_t>& data, size_t offset)
{
    return (int16_t)readU16(data, offset);
}

uint64_t readU64(const std::vector<uint8_t>& data, size_t offset)
{
    uint64_t value = 0;
    for (int b = 0; b < 8; ++b)
    {
        value |= ((uint64_t)data[offset + b]) << (8 * b);
    }
    return value;
}

float readFloat(const std::vector<uint8_t>& data, size_t offset)
{
    uint32_t bits =
        ((uint32_t)data[offset]) |
        ((uint32_t)data[offset + 1] << 8) |
        ((uint32_t)data[offset + 2] << 16) |
        ((uint32_t)data[offset + 3] << 24);

    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

double nsToSeconds(uint64_t ns)
{
    return (double)ns * 1e-9;
}

bool isPacketStart(uint8_t value)
{
    return value == LIDAR_TYPE || value == ACCEL_TYPE || value == ENCODER_TYPE;
}

size_t packetSize(uint8_t type)
{
    switch (type)
    {
        case LIDAR_TYPE: return LIDAR_PACKET_SIZE;
        case ACCEL_TYPE: return ACCEL_PACKET_SIZE;
        case ENCODER_TYPE: return ENCODER_PACKET_SIZE;
        default: return 0;
    }
}

std::vector<uint8_t> readDumpBytes(const std::string& path)
{
    std::ifstream in(path);
    std::vector<uint8_t> bytes;
    std::string token;

    while (in >> token)
    {
        unsigned int value;
        std::stringstream ss;
        ss << std::hex << token;

        if (ss >> value)
        {
            bytes.push_back((uint8_t)value);
        }
    }

    return bytes;
}

bool looksLikeLidar(const std::vector<uint8_t>& bytes, size_t offset)
{
    return offset + LIDAR_PACKET_SIZE <= bytes.size() &&
           bytes[offset] == LIDAR_TYPE &&
           bytes[offset + 1] == 0x2C;
}

bool looksLikeAccel(const std::vector<uint8_t>& bytes, size_t offset)
{
    return offset + ACCEL_PACKET_SIZE <= bytes.size() && bytes[offset] == ACCEL_TYPE;
}

bool withinAbs(int16_t value, int max_abs)
{
    return value >= -max_abs && value <= max_abs;
}

bool isSaneAccel(const std::vector<uint8_t>& packet)
{
    const int16_t gx = readI16(packet, 1);
    const int16_t gy = readI16(packet, 3);
    const int16_t gz = readI16(packet, 5);
    const int16_t ax = readI16(packet, 7);
    const int16_t ay = readI16(packet, 9);
    const int16_t az = readI16(packet, 11);
    const uint64_t dt = readU64(packet, ACCEL_DT_OFFSET);

    if (dt == 0 || dt > MAX_ACCEL_DT_NS)
    {
        return false;
    }

    if (gx == INT16_MIN || gy == INT16_MIN || gz == INT16_MIN ||
        ax == INT16_MIN || ay == INT16_MIN || az == INT16_MIN)
    {
        return false;
    }

    if (!withinAbs(gx, MAX_GYRO_RAW) ||
        !withinAbs(gy, MAX_GYRO_RAW) ||
        !withinAbs(gz, MAX_GYRO_RAW) ||
        !withinAbs(ax, MAX_ACCEL_AXIS_RAW) ||
        !withinAbs(ay, MAX_ACCEL_AXIS_RAW) ||
        !withinAbs(az, MAX_ACCEL_AXIS_RAW))
    {
        return false;
    }

    const int64_t accelMagnitudeSq =
        (int64_t)ax * ax +
        (int64_t)ay * ay +
        (int64_t)az * az;

    return accelMagnitudeSq >= MIN_ACCEL_MAGNITUDE_SQ &&
           accelMagnitudeSq <= MAX_ACCEL_MAGNITUDE_SQ;
}

bool looksLikeEncoder(const std::vector<uint8_t>& bytes, size_t offset)
{
    if (offset + ENCODER_PACKET_SIZE > bytes.size() || bytes[offset] != ENCODER_TYPE)
    {
        return false;
    }

    uint8_t encoder = bytes[offset + ENCODER_ID_OFFSET];
    return encoder == 1 || encoder == 2;
}

bool alignedAt(const std::vector<uint8_t>& bytes, size_t offset)
{
    return looksLikeLidar(bytes, offset) || looksLikeAccel(bytes, offset) || looksLikeEncoder(bytes, offset);
}

void decodeLidar(const std::vector<uint8_t>& packet,
                 std::vector<double>& distancesMeters,
                 std::vector<double>& anglesDegrees)
{
    distancesMeters.clear();
    anglesDegrees.clear();
    distancesMeters.reserve(POINT_PER_PACK);
    anglesDegrees.reserve(POINT_PER_PACK);

    uint16_t startAngleRaw = readU16(packet, 4);
    uint16_t endAngleRaw = readU16(packet, 42);

    double start = (double)startAngleRaw / 100.0;
    double end = (double)endAngleRaw / 100.0;

    if (end < start)
    {
        end += 360.0;
    }

    for (int i = 0; i < POINT_PER_PACK; i++)
    {
        size_t pointOffset = 6 + i * 3;
        double distance = (double)readU16(packet, pointOffset) / 1000.0; // LD06 distance is mm.
        double angle = start + (end - start) * (double)i / (double)(POINT_PER_PACK - 1);

        while (angle >= 360.0)
        {
            angle -= 360.0;
        }

        distancesMeters.push_back(distance);
        anglesDegrees.push_back(angle);
    }
}

bool decodeAccel(const std::vector<uint8_t>& packet,
                 double& imu_dt,
                 double& gyro,
                 double& accelerationX,
                 double& accelerationY)
{
    if (!isSaneAccel(packet))
    {
        return false;
    }

    accelerationX = (double)readI16(packet, ACCEL_X_OFFSET);
    accelerationY = (double)readI16(packet, ACCEL_Y_OFFSET);
    gyro = (double)readI16(packet, GYRO_Z_OFFSET);
    imu_dt = nsToSeconds(readU64(packet, ACCEL_DT_OFFSET));
    return true;
}

void decodeEncoder(const std::vector<uint8_t>& packet,
                   double& en_dt,
                   double& distanceR,
                   double& distanceL)
{
    uint8_t encoder = packet[ENCODER_ID_OFFSET];
    double meters = (double)readFloat(packet, ENCODER_DISTANCE_OFFSET);
    double dt = nsToSeconds(readU64(packet, ENCODER_DT_OFFSET));

    if (encoder == 1)
    {
        distanceR = meters;
        en_dt = dt;
    }
    else if (encoder == 2)
    {
        distanceL = -meters;
        en_dt = dt;
    }
}

std::string buildMatlabPacket(double en_dt,
                              double distanceR,
                              double distanceL,
                              double imu_dt,
                              double gyro,
                              double accelerationX,
                              double accelerationY,
                              const std::vector<double>& lidarDistances,
                              const std::vector<double>& lidarAngles)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);

    const size_t lidar_n = std::min(lidarDistances.size(), lidarAngles.size());

    // MATLAB expects:
    // [en_dt, distanceR, distanceL, imu_dt, gyro, accelerationX, accelerationY,
    //  lidar_n, lidar_dist_1..n, lidar_ang_1..n]
    oss << en_dt << ","
        << distanceR << ","
        << distanceL << ","
        << imu_dt << ","
        << gyro << ","
        << accelerationX << ","
        << accelerationY << ","
        << lidar_n;

    for (size_t i = 0; i < lidar_n; ++i)
    {
        oss << "," << lidarDistances[i];
    }

    for (size_t i = 0; i < lidar_n; ++i)
    {
        oss << "," << lidarAngles[i];
    }

    oss << "\n";
    return oss.str();
}
}

int main(int argc, char** argv)
{
    std::string dumpPath = argc > 1 ? argv[1] : "dump.txt";
    std::string host = argc > 2 ? argv[2] : "127.0.0.1";
    int port = argc > 3 ? std::atoi(argv[3]) : 5005;
    int delayMs = argc > 4 ? std::atoi(argv[4]) : 50;

    std::vector<uint8_t> dumpBytes = readDumpBytes(dumpPath);
    if (dumpBytes.empty())
    {
        std::cout << "Dump file is empty or not found: " << dumpPath << "\n";
        return -1;
    }

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        std::cout << "WSAStartup failed\n";
        return -1;
    }

    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET)
    {
        std::cout << "Socket creation failed\n";
        WSACleanup();
        return -1;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons((u_short)port);
    inet_pton(AF_INET, host.c_str(), &serverAddr.sin_addr);

    std::cout << "Connecting to MATLAB " << host << ":" << port << "...\n";

    if (connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
    {
        std::cout << "Connection failed\n";
        closesocket(sock);
        WSACleanup();
        return -1;
    }

    std::cout << "Connected!\n";

    double en_dt = 0.0;
    double imu_dt = 0.0;
    double distanceR = 0.0;
    double distanceL = 0.0;
    double gyro = 0.0;
    double accelerationX = 0.0;
    double accelerationY = 0.0;

    std::vector<double> accumulatedLidarDistances;
    std::vector<double> accumulatedLidarAngles;
    accumulatedLidarDistances.reserve(720);
    accumulatedLidarAngles.reserve(720);

    bool haveImu = false;
    bool haveRightEncoder = false;
    bool haveLeftEncoder = false;
    bool insideMotionBlock = false;
    int sentPackets = 0;

    auto sendAccumulatedScan = [&]() -> bool
    {
        if (accumulatedLidarDistances.empty())
        {
            return true;
        }

        if (!haveImu || !haveRightEncoder || !haveLeftEncoder)
        {
            std::cout << "Skipping accumulated LiDAR scan because IMU/right/left encoder data is not ready yet. "
                      << "points=" << accumulatedLidarDistances.size() << "\n";
            accumulatedLidarDistances.clear();
            accumulatedLidarAngles.clear();
            return true;
        }

        std::string matlabPacket = buildMatlabPacket(
            en_dt,
            distanceR,
            distanceL,
            imu_dt,
            gyro,
            accelerationX,
            accelerationY,
            accumulatedLidarDistances,
            accumulatedLidarAngles);

        int sent = send(sock, matlabPacket.c_str(), (int)matlabPacket.size(), 0);
        if (sent == SOCKET_ERROR)
        {
            std::cout << "Send failed\n";
            return false;
        }

        sentPackets++;
        std::cout << "Sent MATLAB packet " << sentPackets
                  << " with " << accumulatedLidarDistances.size() << " LiDAR points\n";

        accumulatedLidarDistances.clear();
        accumulatedLidarAngles.clear();
        Sleep((DWORD)delayMs);
        return true;
    };

    for (size_t i = 0; i < dumpBytes.size();)
    {
        if (!isPacketStart(dumpBytes[i]) || !alignedAt(dumpBytes, i))
        {
            i++;
            continue;
        }

        size_t size = packetSize(dumpBytes[i]);
        if (size == 0 || i + size > dumpBytes.size())
        {
            break;
        }

        std::vector<uint8_t> packet(dumpBytes.begin() + i, dumpBytes.begin() + i + size);

        if (packet[0] == ACCEL_TYPE)
        {
            if (decodeAccel(packet, imu_dt, gyro, accelerationX, accelerationY))
            {
                haveImu = true;
                insideMotionBlock = true;
            }
            else
            {
                std::cout << "Skipping invalid accel packet\n";
            }
        }
        else if (packet[0] == ENCODER_TYPE)
        {
            uint8_t encoder = packet[ENCODER_ID_OFFSET];
            decodeEncoder(packet, en_dt, distanceR, distanceL);
            if (encoder == 1)
            {
                haveRightEncoder = true;
            }
            else if (encoder == 2)
            {
                haveLeftEncoder = true;
            }
            insideMotionBlock = true;
        }
        else if (packet[0] == LIDAR_TYPE)
        {
            if (insideMotionBlock)
            {
                if (!sendAccumulatedScan())
                {
                    break;
                }
                insideMotionBlock = false;
            }

            std::vector<double> lidarDistances;
            std::vector<double> lidarAngles;
            decodeLidar(packet, lidarDistances, lidarAngles);

            accumulatedLidarDistances.insert(accumulatedLidarDistances.end(),
                                             lidarDistances.begin(), lidarDistances.end());
            accumulatedLidarAngles.insert(accumulatedLidarAngles.end(),
                                          lidarAngles.begin(), lidarAngles.end());
        }

        i += size;
    }

    sendAccumulatedScan();

    send(sock, "STOP\n", 5, 0);
    std::cout << "Sent " << sentPackets << " MATLAB packets\n";

    closesocket(sock);
    WSACleanup();

    return 0;
}
