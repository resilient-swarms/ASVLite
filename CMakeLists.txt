CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(asv_simulator C)

# INCLUDE DIRECTORIES
# -------------------
INCLUDE_DIRECTORIES(
  include
  )

# SOURCE FILES
# ------------
SET( SOURCE
  source/regular_wave.c
  source/wave.c
  source/wind.c
  source/current.c
  source/main.c
  )

# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(asv_simulator ${SOURCE})

# LINK LIBRARIES
# --------------
TARGET_LINK_LIBRARIES(asv_simulator m)
