cmake_minimum_required(VERSION 3.0)

set(ProjectName "main")

project(${ProjectName})

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

add_executable(${ProjectName} main.cpp type.h my_udp.cpp my_udp.h
    graph.cpp graph.h graph_scatter.cpp graph_scatter.h graph_object.cpp graph_object.h
    lsm_sphere.cpp lsm_sphere.h
)

if (WIN32)
else()
    target_link_libraries(${ProjectName} pthread)
endif()


find_package(OpenCV REQUIRED)
target_include_directories(${ProjectName} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${ProjectName} ${OpenCV_LIBS})
