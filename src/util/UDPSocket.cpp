#include <iostream>
#include <cstring>
#include "UDPSocket.h"

UDPSocket::UDPSocket(const std::string &ip, uint16_t port)
{
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(0x0101, &wsaData))
    {
        std::cerr << "Failed to initialize Windows socket API" << std::endl;
        return;
    }
#endif

    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
        std::cerr << "Failed to open UDP socket" << std::endl;
        return;
    }

    auto* iNetAddr = (sockaddr_in*) &addr_;
    iNetAddr->sin_family = AF_INET;
    iNetAddr->sin_port = htons(port);
    if(!inet_aton(ip.c_str(), &iNetAddr->sin_addr))
    {
        std::cerr << "Invalid UDP target address " << ip << std::endl;
        return;
    }
}

UDPSocket::~UDPSocket()
{
#ifdef _WIN32
    WSACleanup();
#endif
}

void UDPSocket::send(const std::vector<uint8_t>& data)
{
    size_t size = data.size() < 65508 ? data.size() : 65507;
    if(sendto(socket_, data.data(), size, 0, &addr_, sizeof(addr_)) < 0)
    {
        // TODO: replace by easylogging and only print every N occurances
//        std::cerr << "UDP Frame send failed: " << strerror(errno) << " " << strerrorname_np(errno) << std::endl;
    }
}
