#include <cstdint>

#define POINT_PER_PACK 12
#define ANGLE_PER_FRAME 12
#define HEADER 0x54

typedef struct __attribute__((packed))
{
    uint16_t distanceValue;
    uint8_t confidence;
} DataPoints;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t dataLength;
    uint16_t speed;
    uint16_t startAngle;
    DataPoints point[POINT_PER_PACK];
    uint16_t endAngle;
    uint16_t timestamp;
    uint8_t crc8;
} LidarFrame;
