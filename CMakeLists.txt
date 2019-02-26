cmake_minimum_required(VERSION 3.0.0)

PROJECT(asv_swarm VERSION 0.1.0)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

# Preprocessor macro ENABLE_THREADS for multi-threading
OPTION(ENABLE_VISUALIZATION "Enable visualization" ON) # Enabled by default
IF(ENABLE_VISUALIZATION)
  ADD_DEFINITIONS(-DENABLE_VISUALIZATION)
ENDIF(ENABLE_VISUALIZATION)

# Build configuration
SET(BUILD_CONFIG "Debug" CACHE STRING
    "Build configuration selected at CMake configuration time")
SET_PROPERTY(CACHE BUILD_CONFIG PROPERTY STRINGS Release Debug Test)
IF("${BUILD_CONFIG}" STREQUAL "Release" )
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Find libraries
# VTK
FIND_PACKAGE(VTK REQUIRED)
VTK_MODULE_CONFIG( VTK
  vtkCommonCore
  vtkCommonDataModel
  vtkFiltersGeometry
  vtkInteractionStyle
  vtkRenderingOpenGL2
  vtkInteractionStyle
  vtkRenderingCore
  vtkRenderingFreeType
)
INCLUDE(${VTK_USE_FILE})

# Source files
SET( SOURCE  
  source/main.cpp
  source/geometry.cpp 
  source/regular_wave.cpp
  source/wave_spectrum.cpp
  source/sea_surface_dynamics.cpp
  source/sea_surface_actor.cpp
)

# Additional source files for test build
SET( TEST_SOURCE
  tests/unit_test_runner.cpp
  tests/test_regular_wave.cpp
)

# Include directories
INCLUDE_DIRECTORIES(
  include
)

# Test build requires some additional source files, libs and preprocessor 
# macros and is build as an executable 
#IF("${BUILD_CONFIG}" STREQUAL "Test")
#  ADD_DEFINITIONS(-DTEST)
#  SET( SOURCE ${SOURCE} ${TEST_SOURCE})
#ENDIF()

ADD_EXECUTABLE(asv_swarm ${SOURCE})
TARGET_LINK_LIBRARIES(asv_swarm ${VTK_LIBRARIES})
