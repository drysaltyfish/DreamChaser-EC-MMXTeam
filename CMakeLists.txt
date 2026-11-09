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
set(CMAKE_PROJECT_NAME Fusai)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

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
    # Add user sources here
    Core/Src/driver/motor.c
    Core/Src/driver/uart_connection.c
    Core/Src/tasks/default_task.c
    Core/Src/control/pid.c
    Core/Src/control/vehicle.c

    Core/Src/utils/byte_ring.c
    Core/Src/utils/crc.c

    Core/Src/protocol/protocol_codec.c
    Core/Src/protocol/protocol_decoder.c
    Core/Src/protocol/protocol_messages.c

    Core/Src/motion/gimbal.c
    Core/Src/motion/chassis.c
    Core/Src/motion/motion_arbiter.c
    Core/Src/motion/motion_control.c

    Core/Src/app/calibration.c
    Core/Src/app/gimbal_port.c
    Core/Src/app/chassis_port.c
    Core/Src/app/tx_queue.c
    Core/Src/app/command_router.c
    Core/Src/app/telemetry.c
    Core/Src/app/app.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc/

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
