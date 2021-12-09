cmake_minimum_required(VERSION 3.0)

set(ProjectName "main")

project(${ProjectName})

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

add_executable(${ProjectName} main.cpp my_udp.cpp my_udp.h)

if (WIN32)
else()
    target_link_libraries(${ProjectName} pthread)
endif()

