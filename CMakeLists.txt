cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME dm_mc02)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# C++ language standard (sp_middleware 使用了 C++17 的 inline variable)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # user sources: sp_middleware
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware/io/buzzer/buzzer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware/io/dbus/dbus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware/io/fdcan/fdcan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware/io/plotter/plotter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware/io/ws2812/ws2812.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware/motor/lk_motor/lk_motor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware/tools/math_tools/math_tools.cpp

    # user sources: applications
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/controllers/chassis_task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/data_interfaces/can/can_recv.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/data_interfaces/can/can_send.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/data_interfaces/uart/uart_task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/io/buzzer_task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/io/led_task.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/plotter_task.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sp_middleware
    ${CMAKE_CURRENT_SOURCE_DIR}/applications
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
