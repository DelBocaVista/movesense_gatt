cmake_minimum_required(VERSION 3.13)
project(source)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(source
        wbresources/resources.cpp
        App.cpp
        CustomGATTSvcClient.cpp
        CustomGATTSvcClient.h)
