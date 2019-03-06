cmake_minimum_required(VERSION 3.0.0)

PROJECT(asv_swarm VERSION 0.1.0)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

# Build configurations
MARK_AS_ADVANCED(FORCE CMAKE_BUILD_TYPE)
# Create a enum list for selecting build configuration and set Release as the
# default option
SET(BUILD_CONFIG "Release" CACHE STRING
    "Build configuration selected at CMake configuration time")
SET_PROPERTY(CACHE BUILD_CONFIG PROPERTY STRINGS Release Debug Test)
IF(BUILD_CONFIG MATCHES "Release")
  SET(CMAKE_BUILD_TYPE Release)
ELSEIF(BUILD_CONFIG MATCHES "Debug")
  SET(CMAKE_BUILD_TYPE Debug)
ELSEIF(BUILD_CONFIG MATCHES "Test")
  SET(CMAKE_BUILD_TYPE Test)
ENDIF(BUILD_CONFIG MATCHES "Release")
MESSAGE(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

# Preprocessor macro for enabling visualisation
IF(CMAKE_BUILD_TYPE MATCHES Test)
  OPTION(ENABLE_VISUALISATION "Enable visualisation" OFF) 
ELSE(CMAKE_BUILD_TYPE MATCHES Test)
  OPTION(ENABLE_VISUALISATION "Enable visualisation" ON) 
  ADD_DEFINITIONS(-DENABLE_VISUALISATION)
ENDIF(CMAKE_BUILD_TYPE MATCHES Test)

# Include directories
INCLUDE_DIRECTORIES(
  include
  )

# Source files required for all build configurations
MESSAGE(STATUS "Add source files required for all build configurations.")
SET( SOURCE  
  source/geometry.cpp 
  source/regular_wave.cpp
  source/wave_spectrum.cpp
  source/sea_surface_dynamics.cpp
  )

# Note: visualisation cannot be enabled for Test build.
IF(ENABLE_VISUALISATION AND NOT CMAKE_BUILD_TYPE MATCHES Test)
  # Additional source files for visualisation
  MESSAGE(STATUS "Visualisation enabled")
  MESSAGE(STATUS "Add source files required for visualisation.")
  SET(SOURCE_VISUALISATION 
    source/main.cpp
    source/sea_surface_actor.cpp
    source/scene.cpp
    )
  SET(SOURCE ${SOURCE} ${SOURCE_VISUALISATION})
  # Find and add vtk libraries
  MESSAGE(STATUS "Find VTK libraries required for visualisation.")
  FIND_PACKAGE(VTK REQUIRED)
  VTK_MODULE_CONFIG( VTK
    vtkCommonColor
    vtkCommonCore
    vtkCommonDataModel
    vtkInteractionStyle
    vtkInteractionWidgets
    vtkRenderingAnnotation
    vtkRenderingCore
    vtkRenderingFreeType
    vtkRenderingOpenGL2
    )
  INCLUDE(${VTK_USE_FILE})
ENDIF(ENABLE_VISUALISATION AND NOT CMAKE_BUILD_TYPE MATCHES Test)

# Additional source files for test build
IF(CMAKE_BUILD_TYPE MATCHES Test)
  MESSAGE(STATUS "Build configuraton: Test.")
  MESSAGE(STATUS "Add source files required for unit testing.")
  LIST(REMOVE_ITEM SOURCE "source/main.cpp")
  SET(SOURCE_TEST 
    tests/unit_test_runner.cpp
    tests/test_regular_wave.cpp
    )
  SET(SOURCE ${SOURCE} ${SOURCE_TEST})
  MESSAGE(STATUS "Find Google Test libraries required for unit testing.")
  FIND_PACKAGE(GTest REQUIRED)
ENDIF(CMAKE_BUILD_TYPE MATCHES Test)

# Create binary
IF(ENABLE_VISUALISATION OR CMAKE_BUILD_TYPE MATCHES Test)
  MESSAGE(STATUS "Create asv_swarm executable.")
  ADD_EXECUTABLE(asv_swarm ${SOURCE})
ELSE(ENABLE_VISUALISATION OR CMAKE_BUILD_TYPE MATCHES Test)
  MESSAGE(STATUS "Create asv_swarm library.")
  ADD_LIBRARY(asv_swarm ${SOURCE})
ENDIF(ENABLE_VISUALISATION OR CMAKE_BUILD_TYPE MATCHES Test)

# Link visualisation libraries
IF(ENABLE_VISUALISATION AND NOT CMAKE_BUILD_TYPE MATCHES Test)
  MESSAGE(STATUS "Link VTK libraries.")
  TARGET_LINK_LIBRARIES(asv_swarm ${VTK_LIBRARIES})
# Link google test libraries
ELSEIF(CMAKE_BUILD_TYPE MATCHES Test)
  MARK_AS_ADVANCED(CLEAR GTEST_LIBRARY GTEST_MAIN_LIBRARY GTEST_INCLUDE_DIR)
  MESSAGE(STATUS "Link Google Test libraries.")
  TARGET_LINK_LIBRARIES(asv_swarm 
                        ${GTEST_LIBRARY} 
                        ${GTEST_MAIN_LIBRARY}
                        -lpthread)
ENDIF(ENABLE_VISUALISATION AND NOT CMAKE_BUILD_TYPE MATCHES Test)
