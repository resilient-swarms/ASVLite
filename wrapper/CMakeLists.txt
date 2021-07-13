CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

PROJECT(ASVLite-python LANGUAGES C VERSION 0.0.1 DESCRIPTION "Python wrapper for ASVLite")

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
OPTION(ENABLE_TIME_SYNC "Enable time sync between parallel threads." OFF) # Disabled by default
IF(ENABLE_MULTI_THREADING)
  IF(ENABLE_TIME_SYNC)
    ADD_DEFINITIONS(-DENABLE_TIME_SYNC)
    MESSAGE(STATUS "Time sycn between threads enabled.")
  ELSE()
    MESSAGE(STATUS "Time sycn between threads disabled.")
  ENDIF(ENABLE_TIME_SYNC)
ENDIF(ENABLE_MULTI_THREADING)

# ENABLE EARTH-COORDINATES
# ------------------------
OPTION(ENABLE_EARTH_COORDINATES "Enable earth coordinates." ON) # Enabled by default
IF(ENABLE_EARTH_COORDINATES)
  ADD_DEFINITIONS(-DENABLE_EARTH_COORDINATES)
  MESSAGE(STATUS "Earth coordinates enabled.")
ELSE()
  MESSAGE(STATUS "Earth coordinates disabled.")
endif(ENABLE_EARTH_COORDINATES)

# INCLUDE HEADER FILES DIRECTORIES
# --------------------------------
INCLUDE_DIRECTORIES(
  ../include
  ../dependency/tomlc99
  include
  )

# INCLUDE FILES
SET( INCLUDE
  ../include/geometry.h
  ../include/regular_wave.h  
  ../include/wave.h 
  ../include/asv.h 
)

# SOURCE FILES
# ------------
# simulator source files
SET( SOURCE
  ../dependency/tomlc99/toml.c
  ../source/regular_wave.c
  ../source/wave.c
  ../source/asv.c
  )

# CREATE BINARIES
# ---------------
SET(TARGET_DIR ${PROJECT_SOURCE_DIR}/lib)
ADD_LIBRARY(ASVLite-python SHARED ${INCLUDE} ${SOURCE})
FIND_PACKAGE(Threads REQUIRED)
TARGET_LINK_LIBRARIES(ASVLite-python m Threads::Threads)
SET_TARGET_PROPERTIES(ASVLite-python PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "${INCLUDE}")
INSTALL(TARGETS ASVLite-python
    LIBRARY DESTINATION ${TARGET_DIR}
    PUBLIC_HEADER DESTINATION ${TARGET_DIR})
