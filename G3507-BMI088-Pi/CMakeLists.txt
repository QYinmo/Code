cmake_minimum_required(VERSION 3.16)
project(gimbal_rpi VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GIMBAL_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)

function(gimbal_enable_warnings target)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang" OR
       CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        if(GIMBAL_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

find_package(Threads REQUIRED)

add_library(gimbal_core
    src/config.cpp
    src/bmi088.cpp
    src/mahony_filter.cpp
    src/pid_controller.cpp
    src/gimbal_protocol.cpp
    src/run_mode_policy.cpp
    src/uart_port.cpp
    src/uart_tx_state_machine.cpp
    src/safety_state.cpp
    src/signal_stop.cpp
    src/application.cpp)
target_include_directories(gimbal_core PUBLIC include)
target_link_libraries(gimbal_core PUBLIC Threads::Threads)
gimbal_enable_warnings(gimbal_core)

add_executable(gimbal_controller src/main.cpp)
target_link_libraries(gimbal_controller PRIVATE gimbal_core)
gimbal_enable_warnings(gimbal_controller)

add_library(mspm0_gimbal_protocol STATIC
    mspm0_protocol/gimbal_protocol.c
    mspm0_protocol/gimbal_stream_parser.c
    mspm0_protocol/gimbal_safety_monitor.c)
target_include_directories(mspm0_gimbal_protocol PUBLIC mspm0_protocol)
gimbal_enable_warnings(mspm0_gimbal_protocol)

include(CTest)
if(BUILD_TESTING)
    foreach(test_name IN ITEMS crc protocol attitude_uart imu_conversion pid mahony uart_tx safety_state config signal exit_status)
        add_executable(test_${test_name} tests/test_${test_name}.cpp)
        target_link_libraries(test_${test_name} PRIVATE gimbal_core)
        gimbal_enable_warnings(test_${test_name})
        add_test(NAME ${test_name} COMMAND test_${test_name})
    endforeach()
    foreach(test_name IN ITEMS mspm0_stream_parser mspm0_safety)
        add_executable(test_${test_name} tests/test_${test_name}.cpp)
        target_link_libraries(test_${test_name}
            PRIVATE gimbal_core mspm0_gimbal_protocol)
        gimbal_enable_warnings(test_${test_name})
        add_test(NAME ${test_name} COMMAND test_${test_name})
    endforeach()
endif()
