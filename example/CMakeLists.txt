CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

PROJECT(example_1 C)

# INCLUDE HEADER FILES DIRECTORIES
# --------------------------------
INCLUDE_DIRECTORIES(
  ../include
  ../dependency/tomlc99
  )

# SOURCE FILES
# ------------
SET( SOURCE
  ./example_1.c
  ../dependency/tomlc99/toml.c
  ../source/errors.c
  ../source/geometry.c
  ../source/regular_wave.c
  ../source/sea_surface.c
  ../source/asv.c
  )

# PREPROCESSOR MACROS
# -------------------
IF(WIN32)
  ADD_COMPILE_DEFINITIONS(WIN32)
ENDIF()

# CREATE BINARIES
# ---------------
ADD_EXECUTABLE(example_1 ${SOURCE})
SET_PROPERTY(TARGET example_1 PROPERTY C_STANDARD 11)

# LINK LIBRARIES 
# --------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
IF(UNIX)
  TARGET_LINK_LIBRARIES(example_1 m Threads::Threads)
ENDIF()
