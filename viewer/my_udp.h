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
#ifndef MY_UDP_
#define MY_UDP_

/*** Include ***/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>

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


class MyUdp {
public:
    enum class Mode {
        kModeSend = 0,
        kModeRecvBlocking,
        kModeRecvNonBlocking,
    };

public:
    MyUdp(std::string address = "127.0.0.1", uint16_t port = 1234, Mode mode = Mode::kModeSend);
    ~MyUdp();

    void Send(const char* buf, int32_t len);
    void Recv(char* buf, int32_t len);

private:
    static std::mutex mtx_;
    static int32_t num_instance_;

private:
    struct sockaddr_in addr_;
#ifdef WIN32
    SOCKET sock_;
#else
    int32_t sock_;
#endif
};

#endif
