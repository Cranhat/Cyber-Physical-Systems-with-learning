#include "TCPserver.h"

#define PORT "3000"

int main()
{
    bool proc = true;

    TCPserver server(PORT);
    
    if (!server.connectSock())
    {
        return -1;
    }

    while(server.receiveData()) {}

    return 0;
}
