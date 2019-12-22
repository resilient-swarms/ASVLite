CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(asv_simulator C)

# INCLUDE HEADER FILES DIRECTORIES
# --------------------------------
INCLUDE_DIRECTORIES(
  include
  dependency/tomlc99
  )

# SOURCE FILES
# ------------
SET( SOURCE
  dependency/tomlc99/toml.c
  source/io.c
  source/regular_wave.c
  source/wave.c
  source/asv.c
  source/main_regular_wave.c
  source/pid_controller.c
  )

# PREPROCESSOR MACROS
# -------------------
IF(WIN32)
  ADD_COMPILE_DEFINITIONS(WIN32)
ENDIF()

# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(asv_simulator ${SOURCE})
SET_PROPERTY(TARGET asv_simulator PROPERTY C_STANDARD 11)

# LINK LIBRARIES 
# --------------
IF(UNIX)
  TARGET_LINK_LIBRARIES(asv_simulator m)
ENDIF()
