CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(ASVLite-cyclone C)

# ENABLE MULTI-THREADING
# --------------------------------------
OPTION(ENABLE_MULTI_THREADING "Enable multi-threading." ON) # Enabled by default
IF(NOT ENABLE_MULTI_THREADING)
  ADD_DEFINITIONS(-DDISABLE_MULTI_THREADING)
  MESSAGE(STATUS "Multi-threading disabled.")
ELSE()
  MESSAGE(STATUS "Multi-threading enabled.")
endif(NOT ENABLE_MULTI_THREADING)

# ENABLE TIME SYNC WHILE MULTI-THREADING
# --------------------------------------
OPTION(ENABLE_TIME_SYNC "Enable time sync between parallel threads." OFF) # Desabled by default
IF(ENABLE_MULTI_THREADING)
  IF(ENABLE_TIME_SYNC)
    ADD_DEFINITIONS(-DENABLE_TIME_SYNC)
    MESSAGE(STATUS "Time sycn between threads enabled.")
  ELSE()
    MESSAGE(STATUS "Time sycn between threads disabled.")
  ENDIF(ENABLE_TIME_SYNC)
ENDIF(ENABLE_MULTI_THREADING)

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
  ../source/pid_controller.c
  )
# cyclone source files
SET( SOURCE
  ${SOURCE}
  source/cyclone.c
)

# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(ASVLite-cyclone ${SOURCE})
SET_PROPERTY(TARGET ASVLite-cyclone PROPERTY C_STANDARD 11)

# LINK LIBRARIES 
# --------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(netCDF CONFIG REQUIRED)
IF(UNIX)
  TARGET_LINK_LIBRARIES(ASVLite-cyclone m Threads::Threads netcdf)
ENDIF()
