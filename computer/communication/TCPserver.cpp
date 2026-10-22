#include "TCPserver.h"

TCPserver::TCPserver(const char* PORT) : PORT(PORT), sfd(-1), cfd(-1)
{
    resetPacketDump();

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0)
    {
        printf("WSAStartup failed\n");
    };
#endif
}

TCPserver::~TCPserver()
{
#ifdef _WIN32
    closesocket(cfd);
    closesocket(sfd);

    WSACleanup();
#else
    close(cfd);
    close(sfd);
#endif
}

bool TCPserver::AccelHandl()
{
    AccelData data(cfd, sfd, ACCEL_TYPE);

    return data.get_output();
}

bool TCPserver::LidarHandl()
{
    LidarData data(cfd, sfd, LIDAR_TYPE);

    return data.get_output();
}

bool TCPserver::EncoderHandl()
{
    EncoderData data(cfd, sfd, ENCODER_TYPE);

    return data.get_output();
}

bool TCPserver::connectSock()
{
    int status;
    struct addrinfo hints{},*servinfo, *p;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    if ((status = getaddrinfo(NULL, PORT, &hints, &servinfo)) != 0)
    {
        perror("getaddrinfo");
        return false;
    }

    int yes = 1;

    for(p = servinfo; p != NULL; p = p->ai_next)
    {
        if ((sfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1)
        {
            perror("socket");
            continue;
        }

        printf("Server socket\n");

        if (setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, (const char*) & yes, sizeof(int)) == -1)
        {
            perror("setsockopt");
            return false;
        }

        if (bind(sfd, p->ai_addr, p->ai_addrlen) == -1)
        {
#ifdef _WIN32
            closesocket(sfd);

            WSACleanup();
#else
            close(sfd);
#endif

            perror("server: bind");
            continue;
        }

        break;
    }

    freeaddrinfo(servinfo);

    if (p == NULL)
    {
        perror("server: failed to bind");
        return false;
    }

    if (listen(sfd, BACKLOG) == -1)
    {
        perror("listen");
        return false;
    }

    printf("server: waiting to connect...\n");

    struct sockaddr_in cAddr{};

#ifdef _WIN32
    int sin_size = sizeof(cAddr);
#else
    socklen_t sin_size = sizeof cAddr;
#endif

    cfd = accept(sfd,(struct sockaddr *)&cAddr, &sin_size);

    if (cfd == -1)
    {
        perror("accept");
        return false;
    }

    printf("Accepted socket\n");

    return true;
}

bool TCPserver::receiveData()
{
    while (true)
    {
        uint8_t type;
        int ret;

#ifdef _WIN32
        ret = recv(cfd, (char*)&type, sizeof(type), 0);

        if (ret < 0)
        {
            printf("Lidar recv error: %d\n", WSAGetLastError());
            closesocket(cfd);
            closesocket(sfd);

            WSACleanup();
#else
        ret = recv(cfd, &type, sizeof(uint8_t), 0);

        if (ret < 0)
        {
            perror("type recv");
            close(cfd);
            close(sfd);
#endif

            return false;

        } else if (ret == 0){

            return false;
        }

        bool output;

        switch(type)
        {
            case LIDAR_TYPE: output = LidarHandl(); break;
            case ACCEL_TYPE: output = AccelHandl(); break;
            case ENCODER_TYPE: output = EncoderHandl(); break;
            default: continue;
        }

        if (!output)
        {
            return false;
        }
    }
}

