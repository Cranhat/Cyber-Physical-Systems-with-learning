#include "TCPserver.h"

TCPserver::TCPserver(const char* PORT) : cfd(-1), sfd(-1), PORT(PORT)
{
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
    closeSock();

#ifdef _WIN32
    WSACleanup();
#endif
}

bool TCPserver::LidarHandl()
{
    LidarData data(cfd, sfd, LIDAR_TYPE);

    if (data.get_output() && data.is_valid() && onLidar)
    {
        onLidar(data.get_data());
    }

    return data.get_output();
}

bool TCPserver::MotionHandl()
{
    MotionData data(cfd);

    if (data.get_output() && data.is_valid())
    {
        statusSlot = true;  // ANGLE is the last packet of a robot loop
        if (onMotion)
        {
            onMotion(data.get_data());
        }
    }

    return data.get_output();
}

bool TCPserver::ResponseHandl()
{
    ResponseData data(cfd);

    if (data.get_output() && data.is_valid())
    {
        statusSlot = true;
        if (onResponse)
        {
            onResponse(data.get_data());
        }
    }

    return data.get_output();
}

bool TCPserver::StateHandl()
{
    StateData data(cfd);

    if (data.get_output() && data.is_valid())
    {
        statusSlot = true;
        if (onState)
        {
            onState(data.get_data());
        }
    }

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

    socket_t accepted = accept(sfd,(struct sockaddr *)&cAddr, &sin_size);

    if (accepted == (socket_t)-1)
    {
        perror("accept");
        return false;
    }

    char ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &cAddr.sin_addr, ip, sizeof(ip));

    std::lock_guard<std::mutex> lock(sockMutex);
    cfd = accepted;
    peerAddr = std::string(ip) + ":" + std::to_string(ntohs(cAddr.sin_port));

    printf("Accepted socket from %s\n", peerAddr.c_str());

    return true;
}

bool TCPserver::sendData(const uint8_t* data, size_t size)
{
    std::lock_guard<std::mutex> lock(sockMutex);

    if (cfd == (socket_t)-1)
    {
        return false;
    }

    size_t sent = 0;

    while (sent < size)
    {
#ifdef _WIN32
        int ret = send(cfd, (const char*)data + sent, (int)(size - sent), 0);
#else
        int ret = (int)send(cfd, data + sent, size - sent, MSG_NOSIGNAL);
#endif
        if (ret <= 0)
        {
            return false;
        }

        sent += ret;
    }

    return true;
}

void TCPserver::closeSock()
{
    std::lock_guard<std::mutex> lock(sockMutex);

#ifdef _WIN32
    if (cfd != (socket_t)-1) closesocket(cfd);
    if (sfd != (socket_t)-1) closesocket(sfd);
#else
    if (cfd != -1) close(cfd);
    if (sfd != -1) close(sfd);
#endif

    cfd = (socket_t)-1;
    sfd = (socket_t)-1;
    peerAddr.clear();
}

void TCPserver::shutdownSock()
{
    std::lock_guard<std::mutex> lock(sockMutex);

#ifdef _WIN32
    // on Windows only closing wakes a blocked accept()/recv()
    if (cfd != (socket_t)-1) { closesocket(cfd); cfd = (socket_t)-1; }
    if (sfd != (socket_t)-1) { closesocket(sfd); sfd = (socket_t)-1; }
#else
    if (cfd != -1) shutdown(cfd, SHUT_RDWR);
    if (sfd != -1) shutdown(sfd, SHUT_RDWR);
#endif
}

std::string TCPserver::peer()
{
    std::lock_guard<std::mutex> lock(sockMutex);

    return peerAddr;
}

bool TCPserver::receiveData()
{
    statusSlot = true;  // the first bytes of a connection are a robot loop start

    while (true)
    {
        uint8_t type;
        int ret;

#ifdef _WIN32
        ret = recv(cfd, (char*)&type, sizeof(type), 0);
#else
        ret = recv(cfd, &type, sizeof(uint8_t), 0);
#endif

        if (ret < 0)
        {
#ifdef _WIN32
            printf("type recv error: %d\n", WSAGetLastError());
#else
            perror("type recv");
#endif
            return false;
        }
        else if (ret == 0)
        {
            return false;
        }

        bool output;
        const bool atLoopStart = statusSlot;
        statusSlot = false;  // set again by a handler that takes a packet ending a loop / status block

        switch(type)
        {
            case RESPONSE_TYPE: if (!atLoopStart) continue; output = ResponseHandl(); break;
            case STATE_TYPE: if (!atLoopStart) continue; output = StateHandl(); break;
            case LIDAR_TYPE: output = LidarHandl(); break;
            case VELOCITY_TYPE: output = MotionHandl(); break;  // takes the ANGLE packet after it too
            default: continue;
        }

        if (!output)
        {
            return false;
        }
    }
}

