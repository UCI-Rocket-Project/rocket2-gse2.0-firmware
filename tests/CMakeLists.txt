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

# Define the test executables
add_executable(scheduler_tests test_task_scheduler.cpp)
add_executable(adc_tests test_adc_max11614.cpp)

# Link Google Test to executables
target_link_libraries(scheduler_tests PRIVATE gtest_main gtest pthread)
target_link_libraries(adc_tests PRIVATE gtest_main gtest pthread)

# Map dependencies to each test suite
target_include_directories(scheduler_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(adc_tests PRIVATE 
  ${CMAKE_SOURCE_DIR} 
  ${CMAKE_CURRENT_SOURCE_DIR}/mock_hal 
  ${CMAKE_SOURCE_DIR}/RocketDrivers/adc_max11614_i2c 
  ${CMAKE_SOURCE_DIR}/Core/Inc
)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(scheduler_tests)
gtest_discover_tests(adc_tests)