cmake_minimum_required(VERSION 3.10)
project(PCAN_Motion_Interface CXX)
find_package(Threads REQUIRED)

# Use an optimized build for verification unless a build type was requested.
# Multi-config generators choose their configuration at build time.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

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
target_compile_definitions(pcan_demo PRIVATE RTMC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# --- Tests -----------------------------------------------------------------
enable_testing()

add_executable(rtmc_tests
    tests/test_kinematics.cpp
    drive/src/cDriveController.cpp
    drive/src/cDriveCalculator.cpp
)

add_test(NAME kinematics COMMAND rtmc_tests)
target_link_libraries(rtmc_tests PRIVATE Threads::Threads)

add_executable(collision_tests
    tests/test_collision.cpp
    PCAN/src/cCANDriveHandler.cpp
    drive/src/cDriveController.cpp
    drive/src/cDriveCalculator.cpp
    ${COLLISION_SOURCES}
)
target_link_libraries(collision_tests PRIVATE Threads::Threads)

add_test(NAME collision_avoidance COMMAND collision_tests)

# Runtime acceptance launches the real application and owns localhost:8082.
find_program(RTMC_PYTHON_EXECUTABLE NAMES python3)
if(RTMC_PYTHON_EXECUTABLE)
    add_test(NAME ros2_description_fk COMMAND ${RTMC_PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/ros2/src/rtmc_description/test/test_description_fk.py)
    set_tests_properties(ros2_description_fk PROPERTIES TIMEOUT 30)
    add_test(NAME ros2_policy COMMAND ${CMAKE_COMMAND} -E env
        "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/ros2/src/rtmc_ros2"
        ${RTMC_PYTHON_EXECUTABLE} -m unittest discover
        -s ${CMAKE_CURRENT_SOURCE_DIR}/ros2/src/rtmc_ros2/test -p "test_*.py")
    set_tests_properties(ros2_policy PROPERTIES TIMEOUT 15)
    add_test(NAME runtime_avoidance COMMAND ${RTMC_PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_runtime.py --binary $<TARGET_FILE:pcan_demo>)
    set_tests_properties(runtime_avoidance PROPERTIES RUN_SERIAL TRUE TIMEOUT 40)
    add_test(NAME command_source COMMAND ${RTMC_PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_command_source.py --binary $<TARGET_FILE:pcan_demo>)
    set_tests_properties(command_source PROPERTIES RUN_SERIAL TRUE TIMEOUT 15)
    add_test(NAME ros2_adapter_runtime COMMAND ${RTMC_PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ros2_adapter_runtime.py --binary $<TARGET_FILE:pcan_demo>)
    set_tests_properties(ros2_adapter_runtime PROPERTIES RUN_SERIAL TRUE TIMEOUT 15)
    add_test(NAME ros2_action_runtime COMMAND ${RTMC_PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_ros2_action_runtime.py --binary $<TARGET_FILE:pcan_demo>)
    set_tests_properties(ros2_action_runtime PROPERTIES RUN_SERIAL TRUE TIMEOUT 20)
endif()
