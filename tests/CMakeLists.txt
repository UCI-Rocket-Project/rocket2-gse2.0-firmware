project(RocketFirmwareTests CXX)

# Enable std::optional support and C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch Google Test automatically
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
FetchContent_MakeAvailable(googletest)

# Define the test executable
add_executable(scheduler_tests test_task_scheduler.cpp)

# Link Google Test to executable
target_link_libraries(scheduler_tests PRIVATE gtest_main gtest pthread)

# Map the root directory so the test can find your core headers (e.g., task_scheduler.h)
target_include_directories(scheduler_tests PRIVATE ${CMAKE_SOURCE_DIR})

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(scheduler_tests)