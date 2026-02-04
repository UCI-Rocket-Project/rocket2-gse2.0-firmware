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

file(GLOB_RECURSE ALL_DRIVER_SOURCES 
    "RocketDrivers/*.c"
    "RocketDrivers/*.cc"
    "RocketDrivers/*.cpp"
)

# B. Recursively find all directories that contain .h files
# (This allows you to include "header.h" directly, even if it's deep in a folder)
file(GLOB_RECURSE ALL_DRIVER_HEADERS "RocketDrivers/*.h")
set(ALL_DRIVER_INC_DIRS "")
foreach(_headerFile ${ALL_DRIVER_HEADERS})
    get_filename_component(_dir ${_headerFile} DIRECTORY)
    list(APPEND ALL_DRIVER_INC_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES ALL_DRIVER_INC_DIRS)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${ALL_DRIVER_SOURCES}
    Core/Src/main.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${ALL_DRIVER_INC_DIRS}
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
