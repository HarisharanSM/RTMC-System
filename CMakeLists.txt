cmake_minimum_required(VERSION 3.10)
project(PCAN_Motion_Interface CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Cleaned up search directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/PCAN/include
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemController/include
)

add_executable(pcan_demo
    main.cpp
    PCAN/src/cPCANSender.cpp
    PCAN/src/cPCANReceiver.cpp
    PCAN/src/cPCANController.cpp
    SystemController/src/cControlManager.cpp
)