/* Copyright 2020 iwatake2222

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
/*** Include ***/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>

#include <cerrno>

#ifdef WIN32
#include <WinSock2.h> //windows
#pragma comment(lib, "ws2_32.lib")
#else
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "my_udp.h"

/*** Global variable ***/
int32_t MyUdp::num_instance_;
std::mutex MyUdp::mtx_;

/*** Function ***/
MyUdp::MyUdp(std::string address, uint16_t port, MyUdp::Mode mode)
{
    {
        std::lock_guard<std::mutex> lock(mtx_);
#ifdef WIN32
        if (MyUdp::num_instance_ == 0) {
            WSAData wsaData;
            (void)WSAStartup(MAKEWORD(2, 0), &wsaData);
        }
#endif
        MyUdp::num_instance_++;
    }

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    addr_.sin_family = AF_INET;
#ifdef WIN32
    addr_.sin_addr.S_un.S_addr = inet_addr(address.c_str());
#else
    addr_.sin_addr.s_addr = inet_addr(address.c_str());
#endif
    addr_.sin_port = htons(port);

    if (mode == Mode::kModeRecvBlocking || mode == Mode::kModeRecvNonBlocking) {
        bind(sock_, (const struct sockaddr*)&addr_, sizeof(addr_));
        u_long val = (mode == Mode::kModeRecvBlocking) ? 0 : 1;
#ifdef WIN32
        ioctlsocket(sock_, FIONBIO, &val);
#else
        ioctl(sock_, FIONBIO, &val);
#endif
        errno = 0;
    }
}

MyUdp::~MyUdp()
{
#ifdef WIN32
    closesocket(sock_);
#else
    close(sock_);
#endif

    {
        std::lock_guard<std::mutex> lock(mtx_);
        MyUdp::num_instance_--;
#ifdef WIN32
        if (MyUdp::num_instance_ == 0) {
            (void)WSACleanup();
        }
#endif
    }

}

void MyUdp::Send(const char* buf, int32_t len)
{
    int32_t ret = sendto(sock_, buf, len, 0, (struct sockaddr*)&addr_, sizeof(addr_));
    if (ret < 0) {
        printf("[MyUdp::Send][ERROR] ret = %d, errno = %d\n", ret, errno);
    }
}

void MyUdp::Recv(char* buf, int32_t len)
{
    int32_t ret = recv(sock_, buf, len, 0);
    if (ret < 0) {
        printf("[MyUdp::Recv][ERROR] ret = %d, errno = %d\n", ret, errno);
    }
}
