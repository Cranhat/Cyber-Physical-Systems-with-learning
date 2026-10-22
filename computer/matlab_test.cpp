#include <winsock2.h>
#include <ws2tcpip.h>

#include <iostream>
#include <cmath>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

struct Pillar
{
    double x;
    double y;
    double radius;
};

std::vector<Pillar> pillars = {
    // Pillars around the room, kept away from the robot circular path.
    // Robot circle: center (0, 2.5), radius 1.0
    { -1.55, -0.45, 0.18 },
    {  1.55, -0.45, 0.18 },
    { -1.55,  0.95, 0.16 },
    {  1.55,  0.95, 0.16 },
    { -1.55,  4.05, 0.16 },
    {  1.55,  4.05, 0.16 },
    { -1.55,  5.45, 0.18 },
    {  1.55,  5.45, 0.18 },
    {  0.00,  5.45, 0.16 },
    {  0.00, -0.45, 0.16 },
    {  0.00,  2.50, 0.16 }
};

int main()
{
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(5005);

    inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr);

    std::cout << "Connecting to MATLAB...\n";

    if (connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
    {
        std::cout << "Connection failed\n";
        return -1;
    }

    std::cout << "Connected!\n";

    // =========================
    // Fake robot parameters
    // =========================
    double dt = 0.2;
    double L = 0.05;

    // Start on a circular path.
    // The robot will drive counter-clockwise around this circle.
    const double circleCenterX = 0.0;
    const double circleCenterY = 2.5;
    const double circleRadius  = 1.0;

    double x = circleCenterX + circleRadius;
    double y = circleCenterY;
    double theta = M_PI / 2.0;

    int step = 0;

    // =========================
    // Main loop
    // =========================
    while (true)
    {
        double t = step * dt;

    // Simulated robot motion: fixed circular driving path
    // The pillars are placed around the room, not on this path.
    double baseV = 0.22;
    double v = baseV;

    // For differential-drive circular motion:
    // omega = v / radius
    double omega = baseV / circleRadius;

    double dtheta = omega * dt;
    double ds = v * dt;

    // Wheel distances
    double distanceR = ds + (L / 2.0) * dtheta;
    double distanceL = ds - (L / 2.0) * dtheta;

    // Simulated gyro
    double gyro = omega;

    // Update true pose directly on the circle to keep the path clean
    double circleAngle = omega * t;
    x = circleCenterX + circleRadius * cos(circleAngle);
    y = circleCenterY + circleRadius * sin(circleAngle);

    // Robot heading is tangent to the circle
    theta = circleAngle + M_PI / 2.0;

    // =========================
    // Build TCP packet
    // distanceR,distanceL,gyro,range1,...,range360
    // =========================
    const int numRanges = 360;
    const double maxRange = 8.0;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    oss << distanceR << ","
        << distanceL << ","
        << gyro;

    for (int i = 0; i < numRanges; i++)
    {
        double angle = -M_PI + i * (2.0 * M_PI / (numRanges - 1));
        double beamAngle = theta + angle;

        double dx = cos(beamAngle);
        double dy = sin(beamAngle);

        double range = maxRange;

        // Room walls: x = -2, x = 2, y = -1, y = 6
        std::vector<double> candidates;

        if (fabs(dx) > 1e-6)
        {
            double tx1 = (-2.0 - x) / dx;
            double tx2 = ( 2.0 - x) / dx;

            if (tx1 > 0) candidates.push_back(tx1);
            if (tx2 > 0) candidates.push_back(tx2);
        }

        if (fabs(dy) > 1e-6)
        {
            double ty1 = (-1.0 - y) / dy;
            double ty2 = ( 6.0 - y) / dy;

            if (ty1 > 0) candidates.push_back(ty1);
            if (ty2 > 0) candidates.push_back(ty2);
        }

        for (double hit : candidates)
        {
            double hx = x + hit * dx;
            double hy = y + hit * dy;

            if (hx >= -2.0 && hx <= 2.0 && hy >= -1.0 && hy <= 6.0)
            {
                if (hit < range)
                    range = hit;
            }
        }

        // Pillars
        for (const auto& p : pillars)
        {
            double ox = x - p.x;
            double oy = y - p.y;

            double b = 2.0 * (dx * ox + dy * oy);
            double c = ox * ox + oy * oy - p.radius * p.radius;

            double disc = b * b - 4.0 * c;

            if (disc >= 0.0)
            {
                double sqrtDisc = sqrt(disc);
                double t1 = (-b - sqrtDisc) / 2.0;
                double t2 = (-b + sqrtDisc) / 2.0;

                double hit = maxRange;

                if (t1 > 0.02)
                    hit = t1;
                else if (t2 > 0.02)
                    hit = t2;

                if (hit < range)
                    range = hit;
            }
        }

        range += 0.01 * ((double)rand() / RAND_MAX - 0.5);

        if (range < 0.05) range = 0.05;
        if (range > maxRange) range = maxRange;

        oss << "," << range;
    }

    oss << "\n";

    std::string packet = oss.str();

    // Send to MATLAB
    send(sock, packet.c_str(), (int)packet.size(), 0);

    // Console debug
    std::cout << "Sent packet with "
            << 3 + numRanges
            << " values" << std::endl;

    Sleep((DWORD)(dt * 1000));

    step++;

    // Stop after 60 seconds
    if (step > 150)
    {
        send(sock, "STOP\n", 5, 0);
        std::cout << "Sent packet stop!" << std::endl;
        break;
    }
    }
    std::cout << "Closing socket..." << std::endl;
    closesocket(sock);
    WSACleanup();

    return 0;
}
