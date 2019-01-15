cmake_minimum_required(VERSION 3.0.0)

project(wave_dynamics VERSION 0.1.0)

# Source files
set(source  
  wave_dynamics.cpp
  regular_wave.cpp)

# Create the project build configuration file.
if(unittest)
  set(source ${source} unittest.cpp)
  add_executable(wave_dynamics ${source})
  target_link_libraries(wave_dynamics gtest)
else(unittest)
  add_library(wave_dynamics ${source})
endif(unittest)

