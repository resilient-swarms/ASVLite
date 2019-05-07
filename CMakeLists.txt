cmake_minimum_required(VERSION 3.0.0)

PROJECT(asv_swarm VERSION 0.1.0)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

# PREPROCESSOR MACROS
# -------------------
# Enable visualisation
OPTION(ENABLE_VISUALISATION "Enable visualisation" ON) 
IF(ENABLE_VISUALISATION)  
  ADD_DEFINITIONS(-DENABLE_VISUALISATION)
  MESSAGE(STATUS "Visualisation enabled.")
ENDIF(ENABLE_VISUALISATION)  

# Print wave statistics
OPTION(PRINT_WAVE_STATS "Print wave statistics on standard output" ON) 
IF(PRINT_WAVE_STATS)  
  ADD_DEFINITIONS(-DPRINT_WAVE_STATS)
  MESSAGE(STATUS "Printing wave statistics enabled.")
ENDIF(PRINT_WAVE_STATS)  

# BUILD CONFIGURATIONS
# --------------------
MARK_AS_ADVANCED(FORCE CMAKE_BUILD_TYPE)
# Create an enum list for selecting build configuration and set Release as the
# default option
SET(BUILD_CONFIG "Release" CACHE STRING
    "Build configuration selected at CMake configuration time")
SET_PROPERTY(CACHE BUILD_CONFIG PROPERTY STRINGS Release Debug)
IF(BUILD_CONFIG MATCHES "Release")
  SET(CMAKE_BUILD_TYPE Release)
ELSEIF(BUILD_CONFIG MATCHES "Debug")
  SET(CMAKE_BUILD_TYPE Debug)
ELSE(BUILD_CONFIG MATCHES "Release")
  MESSAGE(WARNING "Incorrect value for BUILD_CONFIG")
ENDIF(BUILD_CONFIG MATCHES "Release")
MESSAGE(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

# INCLUDE DIRECTORIES
# -------------------
INCLUDE_DIRECTORIES(
  include
  dependency/eigen-git-mirror
  )

# SOURCE FILES
# ------------
# Common source files.
MESSAGE(STATUS "Add common source files.")
SET( SOURCE  
  source/main_headless.cpp
  source/geometry.cpp 
  source/regular_wave.cpp
  source/wave_spectrum.cpp
  source/sea_surface_dynamics.cpp
  source/asv_dynamics.cpp
  )
# Visualisation source files.
IF(ENABLE_VISUALISATION)
  MESSAGE(STATUS "Add source files required for visualisation.")
  LIST(REMOVE_ITEM SOURCE source/main_headless.cpp)
  SET(SOURCE 
    ${SOURCE}
    source/main_visualisation.cpp
    source/sea_surface_actor.cpp
    source/scene.cpp
    )
ENDIF(ENABLE_VISUALISATION)

# FIND LIBRARIES
# --------------
# VTK 
IF(ENABLE_VISUALISATION)
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
ENDIF(ENABLE_VISUALISATION)

# CREATE BINARIES
# ---------------
MESSAGE(STATUS "Create asv_swarm executable.")
ADD_EXECUTABLE(asv_swarm ${SOURCE})

# LINK LIBRARIES 
# --------------
# Link visualisation libraries
IF(ENABLE_VISUALISATION)
  MESSAGE(STATUS "Link VTK libraries.")
  TARGET_LINK_LIBRARIES(asv_swarm ${VTK_LIBRARIES})
ENDIF(ENABLE_VISUALISATION)
