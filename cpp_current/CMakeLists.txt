CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

PROJECT(ASVLite CXX)

include_directories(
        include
)

SET(SOURCE
        # source/geometry.cpp
        source/rudder_controller.cpp
        # source/main_thrust_tuning.cpp
        source/main_runtime_performance.cpp
        # source/main_rudder_controller_tuning.cpp
)

ADD_EXECUTABLE(ASVLite ${SOURCE})
SET_PROPERTY(TARGET ASVLite PROPERTY CXX_STANDARD 20)
SET_PROPERTY(TARGET ASVLite PROPERTY CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME} PRIVATE
        Eigen3::Eigen
)