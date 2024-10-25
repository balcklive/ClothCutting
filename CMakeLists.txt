cmake_minimum_required(VERSION 3.10)
project(ClothCutting)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost
find_package(Boost REQUIRED COMPONENTS geometry)

# Include directories
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/Lib)
include_directories(${CMAKE_SOURCE_DIR}/Lib/boost)
include_directories(${CMAKE_SOURCE_DIR}/Lib/clipper)

# Add source files
file(GLOB_RECURSE SOURCES 
    "ClothCutting/*.cpp"
    "ClothCutting/*.h"
    "ClothCutting/*.hpp"
    "Lib/clipper/*.cpp"
    "Lib/clipper/*.hpp"
)

# Create executable
add_executable(ClothCutting ${SOURCES})

# Link libraries
target_link_libraries(ClothCutting ${Boost_LIBRARIES})

# Copy executable to Deploy folder
add_custom_command(TARGET ClothCutting POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ClothCutting> ${CMAKE_SOURCE_DIR}/Deploy/
)
