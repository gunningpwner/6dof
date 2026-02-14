cmake_minimum_required(VERSION 3.14)
project(QuadSim6DOF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
include(FetchContent)

# 1. Fetch Eigen (Math Library)
FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(Eigen3)

# --- Include Directories ---
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${Eigen3_SOURCE_DIR} 
)

# --- Source Files ---
# Recursively find all source files (Good for starting out, explicit lists better later)
file(GLOB_RECURSE SOURCES "src/*.cpp")

# --- Executable ---
add_executable(sim_app ${SOURCES})
target_link_libraries(sim_app Eigen3::Eigen)