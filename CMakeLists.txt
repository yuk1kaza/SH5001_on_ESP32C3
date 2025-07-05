cmake_minimum_required(VERSION 3.28)
project(SH5001_ESP32C3_Example)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(SH5001_ESP32C3_Example
        SH5001.C
        SH5001.h
        SH5001_ESP32C3_Example.ino)
