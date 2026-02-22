cmake_minimum_required(VERSION 3.14)
project(QuadSim6DOF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(QUADSIM_DIR ../quadsim)
# --- Dependencies ---
include(FetchContent)

find_package(Eigen3 REQUIRED)

# --- Include Directories ---
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${QUADSIM_DIR}/lib/controllers/include
    ${QUADSIM_DIR}/lib/filters/include
    ${QUADSIM_DIR}/lib/flight_core/include
    ${QUADSIM_DIR}/lib/buffers/include
)

set(FLIGHT_SOURCES
    ${QUADSIM_DIR}/lib/controllers/src/Estimator.cpp
    ${QUADSIM_DIR}/lib/controllers/src/KinematicAdjudicator.cpp
    ${QUADSIM_DIR}/lib/controllers/src/INDIController.cpp
    ${QUADSIM_DIR}/lib/controllers/src/ExcitationGenerator.cpp
    ${QUADSIM_DIR}/lib/controllers/src/GeometricController.cpp
    # Add INDIController.cpp etc.
)



# --- Source Files ---
# Recursively find all source files (Good for starting out, explicit lists better later)
file(GLOB_RECURSE SOURCES "src/*.cpp")
message(STATUS "Source files found: ${SOURCES}")
# --- Executable ---
add_executable(sim_app 
            ${SOURCES}
            ${FLIGHT_SOURCES}
)
target_compile_options(sim_app PRIVATE -O2)
target_compile_definitions(sim_app PRIVATE SIM)