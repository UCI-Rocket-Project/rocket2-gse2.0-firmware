# Defined as a separate CMake project so CMake doesn't try to build the tests
# using an STM32 toolchain
cmake_minimum_required(VERSION 3.14)
project(RocketFirmwareTests CXX)

# Enable std::optional support and other C++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Fetch Google Test automatically
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
FetchContent_MakeAvailable(googletest)

# Define the test executable using the source file in this directory
add_executable(scheduler_tests test_task_scheduler.cpp)

# Link Google Test to executable
target_link_libraries(scheduler_tests PRIVATE gtest_main gtest pthread)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(scheduler_tests)