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
  ../include
  ../dependency/tomlc99
  include
  )

# SOURCE FILES
# ------------
# simulator source files
SET( SOURCE
  ../dependency/tomlc99/toml.c
  ../source/simulation.c
  ../source/regular_wave.c
  ../source/wave.c
  ../source/asv.c
  )
#visualisation source files
SET( SOURCE
  ${SOURCE}
  source/sea_surface_actor.cpp
  source/asv_actor.cpp
  source/scene.cpp
  source/main.cpp
)


# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(visualisation ${SOURCE})

# LINK LIBRARIES 
# --------------
TARGET_LINK_LIBRARIES(visualisation m ${VTK_LIBRARIES})
vtk_module_autoinit(
  TARGETS visualisation
  MODULES ${VTK_LIBRARIES}
)
