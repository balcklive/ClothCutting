cmake_minimum_required(VERSION 3.10)
project(ClothCutting)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加编译选项
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra)
endif()

# Find Boost
find_package(Boost REQUIRED)

# 设置源文件目录
set(SRC_DIR ${CMAKE_SOURCE_DIR}/ClothCutting)
set(LIB_DIR ${CMAKE_SOURCE_DIR}/Lib)

# Include directories
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${SRC_DIR}
    ${LIB_DIR}
    ${LIB_DIR}/boost
    ${LIB_DIR}/clipper
)

# 收集所有源文件
set(SOURCES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/algorithm/problem.cpp
    ${SRC_DIR}/algorithm/solver.cpp
    ${SRC_DIR}/data/nfpPair.cpp
    ${SRC_DIR}/data/piece.cpp
    ${LIB_DIR}/clipper/clipper.cpp
)

# 收集所有头文件
set(HEADERS
    ${SRC_DIR}/Common.h
    ${SRC_DIR}/Config.h
    ${SRC_DIR}/algorithm/problem.h
    ${SRC_DIR}/algorithm/solver.h
    ${SRC_DIR}/data/nfpPair.h
    ${SRC_DIR}/data/piece.h
    ${SRC_DIR}/data/vector.hpp
    ${SRC_DIR}/utils/boostUtils.hpp
    ${SRC_DIR}/utils/clipperUtils.hpp
    ${SRC_DIR}/utils/myUtils.hpp
    ${LIB_DIR}/clipper/clipper.hpp
    ${LIB_DIR}/libnfporb.hpp
)

# 创建可执行文件
add_executable(ClothCutting ${SOURCES} ${HEADERS})

# 链接库
target_link_libraries(ClothCutting ${Boost_LIBRARIES})

# 创建Deploy目录
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/Deploy)

# 复制可执行文件到Deploy文件夹
add_custom_command(TARGET ClothCutting POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ClothCutting> ${CMAKE_SOURCE_DIR}/Deploy/
)
