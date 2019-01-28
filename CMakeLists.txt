cmake_minimum_required(VERSION 3.0.0)

project(wave_dynamics VERSION 0.1.0)

# Include directories
include_directories(include)

# Source files
set(source  
  source/regular_wave.cpp)

# Unit test files
set(unit_test
  unit_tests/unit_test_runner.cpp)

if(test)
  add_executable(wave_dynamics ${source} ${unit_test})
  target_link_libraries(wave_dynamics gtest)
else(test)
  add_library(wave_dynamics ${source})
endif(test)
