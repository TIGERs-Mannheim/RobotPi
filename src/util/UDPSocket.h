#pragma once
#include "interface/RLEFrame.h"

//Adapted from https://gist.github.com/hostilefork/f7cae3dc33e7416f2dd25a402857b6c6

#ifdef _WIN32
#include <Winsock2.h> // before Windows.h, else Winsock 1 conflict
#include <Ws2tcpip.h> // needed for ip_mreq definition for multicast
#include <Windows.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

class UDPSocket
{
public:
    UDPSocket(const std::string& ip, uint16_t port);
    ~UDPSocket();

    void send(const std::vector<uint8_t>& data);
private:
    int socket_;
    struct sockaddr addr_;
};
