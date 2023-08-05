cmake_minimum_required(VERSION 3.24)
project(vcu_core)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(vcu_core
        src/VcuModel.cpp
        src/VcuModel.h src/VcuParameters.h src/blocks/AppsProcessor.h src/blocks/AppsProcessor.cpp src/blocks/BseProcessor.h src/blocks/BseProcessor.cpp src/util/parameters/FunctionParameter3D.h src/util/parameters/FunctionParameter3D.cpp src/util/parameters/FunctionParameter2D.cpp src/blocks/Stompp.h src/blocks/Stompp.cpp src/blocks/TorqueMap.h src/blocks/TorqueMap.cpp src/blocks/TractionControl.h src/blocks/TractionControl.cpp src/blocks/SoftShutdown.cpp src/util/filters/Timer.h src/util/filters/Timer.cpp src/util/filters/LowPassFilter.h src/util/filters/LowPassFilter.cpp)
