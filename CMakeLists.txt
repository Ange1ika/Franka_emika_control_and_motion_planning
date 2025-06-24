cmake_minimum_required(VERSION 3.10)
project(robot_prj)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# libfranka
set(LIBFRANKA_DIR "${CMAKE_SOURCE_DIR}/libfranka")
include_directories(${LIBFRANKA_DIR}/include)
include_directories(${LIBFRANKA_DIR}/common/include) 
include_directories(${LIBFRANKA_DIR}/include/franka)
include_directories(${LIBFRANKA_DIR}/examples)  # for examples_common.h

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})  # Usually /usr/include/eigen3

# Ruckig
include_directories(${CMAKE_SOURCE_DIR}/ruckig/include)
file(GLOB RUCKIG_SRC "${CMAKE_SOURCE_DIR}/ruckig/src/ruckig/*.cpp")
list(REMOVE_ITEM RUCKIG_SRC
    "${CMAKE_SOURCE_DIR}/ruckig/src/ruckig/cloud_client.cpp"
    "${CMAKE_SOURCE_DIR}/ruckig/src/ruckig/python.cpp"
)

# Executable
add_executable(main
  src/main.cpp
  libfranka/examples/examples_common.cpp
  ${RUCKIG_SRC}
)

# Link
target_link_libraries(main
    ${LIBFRANKA_DIR}/build/libfranka.so
    pthread
)
