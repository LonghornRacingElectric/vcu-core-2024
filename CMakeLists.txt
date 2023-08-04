cmake_minimum_required(VERSION 3.24)
project(vcu_core)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(vcu_core
        src/VcuModel.cpp
        src/VcuModel.h src/VcuParameters.h src/blocks/AppsProcessor.h src/blocks/AppsProcessor.cpp src/blocks/BseProcessor.h src/blocks/BseProcessor.cpp)
