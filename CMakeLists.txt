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
set(CMAKE_PROJECT_NAME rocket2-gse2.0-firmware)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Toolchain split: STM32 HAL compiles firmware main, while native compiles tests
if(CMAKE_C_COMPILER MATCHES "arm-none-eabi")
    message(STATUS "Cross-compiler detected: Building STM32 Firmware")
    
    # Enable ASM exclusively for the STM32 build environment
    enable_language(ASM)
    
    # Create an executable object type
    add_executable(${CMAKE_PROJECT_NAME})

    # Add STM32CubeMX generated sources
    add_subdirectory(cmake/stm32cubemx)

    # Link directories setup
    target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE)

    file(GLOB_RECURSE ALL_DRIVER_SOURCES 
        "RocketDrivers/*.cc"
        "RocketDrivers/*.cpp"
        "RocketDrivers/*.c"
    )

    # Recursively find all directories that contain .h files
    file(GLOB_RECURSE ALL_DRIVER_HEADERS "RocketDrivers/*.h")
    set(ALL_DRIVER_INC_DIRS "")
    foreach(_headerFile ${ALL_DRIVER_HEADERS})
        get_filename_component(_dir ${_headerFile} DIRECTORY)
        list(APPEND ALL_DRIVER_INC_DIRS ${_dir})
    endforeach()
    list(REMOVE_DUPLICATES ALL_DRIVER_INC_DIRS)

    # Add sources to executable
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${ALL_DRIVER_SOURCES}
        Core/Src/main.cpp
        Core/Src/gse_controller.cpp
    )

    # Add include paths
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${ALL_DRIVER_INC_DIRS}
    )

    # Add project symbols (macros)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        STM32F1
    )

    # Remove wrong libob.a library dependency when using cpp files
    list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

    # Add linked libraries
    target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
    )

else()
    message(STATUS "Native compiler detected: configuring unit Ttsts")
    
    # Enable CTest integration for the VS Code UI
    include(CTest)
    
    # Route CMake directly to the tests folder
    add_subdirectory(tests)
endif()
