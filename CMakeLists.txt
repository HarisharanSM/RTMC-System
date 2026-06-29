cmake_minimum_required(VERSION 3.10)
project(PCAN_Motion_Interface CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/PCAN/include
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemController/include
    ${CMAKE_CURRENT_SOURCE_DIR}/CANMocker/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drive/include 
)

add_executable(pcan_demo
    main.cpp
    PCAN/src/cPCANSender.cpp
    PCAN/src/cPCANReceiver.cpp
    PCAN/src/cPCANController.cpp
    PCAN/src/cCANDriveHandler.cpp
    SystemController/src/cControlManager.cpp
    CANMocker/src/cCANMocker.cpp
    drive/src/cDrive.cpp  
    drive/src/cDriveController.cpp
    drive/src/cDriveCalculator.cpp
)