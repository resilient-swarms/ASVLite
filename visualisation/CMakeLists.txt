CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(visualisation)

# FIND LIBRARIES
# --------------
# VTK 
find_package(VTK COMPONENTS 
  vtkCommonColor
  vtkCommonCore
  vtkCommonDataModel
  vtkInteractionStyle
  vtkInteractionWidgets
  vtkRenderingAnnotation
  vtkRenderingCore
  vtkRenderingFreeType
  vtkRenderingOpenGL2 )
if (NOT VTK_FOUND)
  message(SEND_ERROR "${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()
message (STATUS "VTK_VERSION: ${VTK_VERSION}")

# INCLUDE HEADER FILES DIRECTORIES
# --------------------------------
INCLUDE_DIRECTORIES(
  ./include
  ../dependency/tomlc99
  )

# SOURCE FILES
# ------------
SET( SOURCE
  dependency/tomlc99/toml.c
  source/io.c
  source/regular_wave.c
  source/wave.c
  source/asv.c
  source/main_visualisation.cpp
  source/scene.cpp
  #source/sea_surface_actor.cpp
  )

# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(asv_simulator ${SOURCE})

# LINK LIBRARIES 
# --------------
TARGET_LINK_LIBRARIES(asv_simulator m ${VTK_LIBRARIES})
vtk_module_autoinit(
  TARGETS asv_simulator
  MODULES ${VTK_LIBRARIES}
)
