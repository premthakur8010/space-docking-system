#pragma once

#include <iostream>
#include <cstring>
#include <cstdint>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using socket_t = SOCKET;
constexpr socket_t INVALID_SOCK_FD = INVALID_SOCKET;
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
using socket_t = int;
constexpr socket_t INVALID_SOCK_FD = -1;
#endif

class NetSystem
{
public:
    static void Initialize()
    {
#if defined(_WIN32)
        static bool initialized = false;
        if (!initialized)
        {
            WSADATA wsa;
            if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
            {
                std::cerr << "[NET] WSAStartup failed\n";
            }
            initialized = true;
        }
#endif
    }
};

class UdpSocket
{
public:
    UdpSocket() : sock_(INVALID_SOCK_FD)
    {
        NetSystem::Initialize();
    }

    ~UdpSocket()
    {
        Close();
    }

    bool Open()
    {
        Close();
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_ == INVALID_SOCK_FD)
        {
            return false;
        }

        // Allow reuse of local addresses
        SetReuseAddr(true);
        return true;
    }

    bool SetReuseAddr(bool enable = true)
    {
        if (sock_ == INVALID_SOCK_FD)
            return false;
        int opt = enable ? 1 : 0;
#if defined(_WIN32)
        return setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt)) == 0;
#else
        bool ok = (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == 0);
#ifdef SO_REUSEPORT
        setsockopt(sock_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif
        return ok;
#endif
    }

    bool SetNonBlocking(bool nonBlocking)
    {
        if (sock_ == INVALID_SOCK_FD)
            return false;

#if defined(_WIN32)
        u_long mode = nonBlocking ? 1 : 0;
        return ioctlsocket(sock_, FIONBIO, &mode) == 0;
#else
        int flags = fcntl(sock_, F_GETFL, 0);
        if (flags < 0) return false;
        flags = nonBlocking ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
        return fcntl(sock_, F_SETFL, flags) == 0;
#endif
    }

    bool Bind(int port, const char* host = "127.0.0.1")
    {
        if (sock_ == INVALID_SOCK_FD && !Open())
            return false;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));

        if (!host || strcmp(host, "0.0.0.0") == 0)
        {
            addr.sin_addr.s_addr = INADDR_ANY;
        }
        else
        {
            inet_pton(AF_INET, host, &addr.sin_addr);
        }

        if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
        {
            return false;
        }

        return true;
    }

    int SendTo(const void* data, size_t size, const char* host, int port)
    {
        if (sock_ == INVALID_SOCK_FD && !Open())
            return -1;

        sockaddr_in dest{};
        dest.sin_family = AF_INET;
        dest.sin_port = htons(static_cast<uint16_t>(port));
        inet_pton(AF_INET, host, &dest.sin_addr);

        return static_cast<int>(sendto(
            sock_,
            reinterpret_cast<const char*>(data),
            static_cast<int>(size),
            0,
            reinterpret_cast<sockaddr*>(&dest),
            sizeof(dest)
        ));
    }

    bool SetMulticastLoop(bool enable)
    {
        if (sock_ == INVALID_SOCK_FD)
            return false;
#if defined(_WIN32)
        int opt = enable ? 1 : 0;
        return setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_LOOP, (const char*)&opt, sizeof(opt)) == 0;
#else
        u_char loop = enable ? 1 : 0;
        return setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) == 0;
#endif
    }

    bool JoinMulticastGroup(const char* groupIp)
    {
        if (sock_ == INVALID_SOCK_FD)
            return false;
        ip_mreq mreq{};
        inet_pton(AF_INET, groupIp, &mreq.imr_multiaddr.s_addr);
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
#if defined(_WIN32)
        return setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&mreq, sizeof(mreq)) == 0;
#else
        return setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) == 0;
#endif
    }

    int RecvFrom(void* buffer, size_t maxSize)
    {
        if (sock_ == INVALID_SOCK_FD)
            return -1;

        sockaddr_in sender{};
#if defined(_WIN32)
        int senderLen = sizeof(sender);
#else
        socklen_t senderLen = sizeof(sender);
#endif

        int bytes = recvfrom(
            sock_,
            reinterpret_cast<char*>(buffer),
            static_cast<int>(maxSize),
            0,
            reinterpret_cast<sockaddr*>(&sender),
            &senderLen
        );

        return bytes;
    }

    void Close()
    {
        if (sock_ != INVALID_SOCK_FD)
        {
#if defined(_WIN32)
            closesocket(sock_);
#else
            close(sock_);
#endif
            sock_ = INVALID_SOCK_FD;
        }
    }

    bool IsValid() const
    {
        return sock_ != INVALID_SOCK_FD;
    }

private:
    socket_t sock_;
};
