CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(ASVLite_visualisation)

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
  ../source/geometry.c
  ../source/errors.c
  ../source/simulation.c
  ../source/regular_wave.c
  ../source/wave.c
  ../source/asv.c
  ../source/pid_controller.c
  )
# visualisation source files
SET( SOURCE
  ${SOURCE}
  source/sea_surface_actor.cpp
  source/asv_actor.cpp
  source/scene.cpp
  source/main.cpp
)


# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(ASVLite_visualisation ${SOURCE})

# LINK LIBRARIES 
# --------------
TARGET_LINK_LIBRARIES(ASVLite_visualisation m ${VTK_LIBRARIES})
vtk_module_autoinit(
  TARGETS ASVLite_visualisation
  MODULES ${VTK_LIBRARIES}
)
