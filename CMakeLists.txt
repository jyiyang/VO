cmake_minimum_required(VERSION 3.13)
project(VO)

find_package(COLMAP REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 3 REQUIRED)


include_directories(${COLMAP_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${COLMAP_LINK_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_subdirectory(src)

