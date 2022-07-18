CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

PROJECT(ASVLite C)

# ENABLE MULTI-THREADING
# --------------------------------------
OPTION(ENABLE_MULTI_THREADING "Enable multi-threading." ON) # Enabled by default
IF(NOT ENABLE_MULTI_THREADING)
  ADD_DEFINITIONS(-DDISABLE_MULTI_THREADING)
  MESSAGE(STATUS "Multi-threading disabled.")
ELSE()
  MESSAGE(STATUS "Multi-threading enabled.")
endif(NOT ENABLE_MULTI_THREADING)

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
  source/errors.c
  source/regular_wave.c
  source/wave.c
  source/asv.c
  source/pid_controller.c
  source/simulation.c
  source/main.c
  )

# PREPROCESSOR MACROS
# -------------------
IF(WIN32)
  ADD_COMPILE_DEFINITIONS(WIN32)
ENDIF()

# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(asvlite ${SOURCE})
SET_PROPERTY(TARGET asvlite PROPERTY C_STANDARD 11)

# LINK LIBRARIES 
# --------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
IF(UNIX)
  TARGET_LINK_LIBRARIES(asvlite m Threads::Threads)
ENDIF()
