CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

PROJECT(ASVLite CXX)

include_directories(
        include
)

SET(SOURCE
        source/geometry.cpp
        source/regular_wave.cpp
        source/sea_surface.cpp
        source/asv.cpp
        # source/thrust_tuning_data_generation.cpp
        source/main.cpp
)

ADD_EXECUTABLE(ASVLite ${SOURCE})
SET_PROPERTY(TARGET ASVLite PROPERTY CXX_STANDARD 20)
SET_PROPERTY(TARGET ASVLite PROPERTY CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME} PRIVATE
        Eigen3::Eigen
)