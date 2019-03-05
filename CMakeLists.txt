cmake_minimum_required(VERSION 3.0.0)

PROJECT(asv_swarm VERSION 0.1.0)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

# Preprocessor macro for enabling visualisation
OPTION(ENABLE_VISUALISATION "Enable visualisation" ON) 
IF(ENABLE_VISUALISATION)
  ADD_DEFINITIONS(-DENABLE_VISUALISATION)
ENDIF(ENABLE_VISUALISATION)

# Include directories
INCLUDE_DIRECTORIES(
  include
  )

# Source files required for all build configurations
MESSAGE("-- Add source files required for all build configurations.")
SET( SOURCE  
  source/geometry.cpp 
  source/regular_wave.cpp
  source/wave_spectrum.cpp
  source/sea_surface_dynamics.cpp
  )

# Note: visualisation cannot be enabled for Test build.
IF(ENABLE_VISUALISATION AND NOT CMAKE_BUILD_TYPE MATCHES Test)
  # Additional source files for visualisation
  MESSAGE("-- Visualisation enabled")
  MESSAGE("-- Add source files required for visualisation.")
  SET(SOURCE_VISUALISATION 
    source/main.cpp
    source/sea_surface_actor.cpp
    source/scene.cpp
    )
  SET(SOURCE ${SOURCE} ${SOURCE_VISUALISATION})
  # Find and add vtk libraries
  MESSAGE("-- Find VTK libraries required for visualisation.")
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
  MESSAGE("-- Build configuraton: Test.")
  MESSAGE("-- Add source files required for unit testing.")
  LIST(REMOVE_ITEM SOURCE "source/main.cpp")
  SET(SOURCE_TEST 
    tests/unit_test_runner.cpp
    tests/test_regular_wave.cpp
    )
  SET(SOURCE ${SOURCE} ${SOURCE_TEST})
  MESSAGE("-- Find Google Test libraries required for unit testing.")
  FIND_PACKAGE(GTest REQUIRED)
ENDIF(CMAKE_BUILD_TYPE MATCHES Test)

ADD_EXECUTABLE(asv_swarm ${SOURCE})

# Link visualisation libraries
IF(ENABLE_VISUALISATION AND NOT CMAKE_BUILD_TYPE MATCHES Test)
  MESSAGE("-- Link VTK libraries.")
  TARGET_LINK_LIBRARIES(asv_swarm ${VTK_LIBRARIES})
ENDIF(ENABLE_VISUALISATION AND NOT CMAKE_BUILD_TYPE MATCHES Test)

# Link google test libraries
IF(CMAKE_BUILD_TYPE MATCHES Test)
  MARK_AS_ADVANCED(CLEAR GTEST_LIBRARY GTEST_MAIN_LIBRARY)
  MESSAGE("-- Link Google Test libraries.")
  TARGET_LINK_LIBRARIES(asv_swarm ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY})
ENDIF(CMAKE_BUILD_TYPE MATCHES Test)

