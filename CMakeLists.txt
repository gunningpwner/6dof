cmake_minimum_required(VERSION 3.14)
project(QuadSim6DOF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
include(FetchContent)

find_package(Eigen3 REQUIRED)

# --- Include Directories ---
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR} 
)

# --- Source Files ---
# Recursively find all source files (Good for starting out, explicit lists better later)
file(GLOB_RECURSE SOURCES "src/*.cpp")
message(STATUS "Source files found: ${SOURCES}")
# --- Executable ---
add_executable(sim_app ${SOURCES})
