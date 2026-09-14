cmake_minimum_required(VERSION 3.10)
project(PCAN_Motion_Interface CXX)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/PCAN/include
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemController/include
    ${CMAKE_CURRENT_SOURCE_DIR}/CANMocker/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drive/include
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/include
)

set(COLLISION_SOURCES
    collision/src/cCollisionMath.cpp
    collision/src/cSceneRegistry.cpp
    collision/src/cBodyKinematics.cpp
    collision/src/cProximityBackend.cpp
    collision/src/cTrajectoryPredictor.cpp
    collision/src/cCollisionSupervisor.cpp
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
    ${COLLISION_SOURCES}
)
target_link_libraries(pcan_demo PRIVATE Threads::Threads)

# --- Tests -----------------------------------------------------------------
enable_testing()

add_executable(rtmc_tests
    tests/test_kinematics.cpp
    drive/src/cDriveController.cpp
    drive/src/cDriveCalculator.cpp
)

add_test(NAME kinematics COMMAND rtmc_tests)

add_executable(collision_tests
    tests/test_collision.cpp
    PCAN/src/cCANDriveHandler.cpp
    drive/src/cDriveController.cpp
    drive/src/cDriveCalculator.cpp
    ${COLLISION_SOURCES}
)
target_link_libraries(collision_tests PRIVATE Threads::Threads)

add_test(NAME collision_avoidance COMMAND collision_tests)
