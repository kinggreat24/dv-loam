cmake_minimum_required (VERSION 2.8)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/lib/x86_64-linux-gnu/cmake")
set(OpenCV_DIR "/usr/local/share/OpenCV")

project (LidarIris)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(
    ./fftm
)

SET(BUILD_SHARED_LIBS ON)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)

add_library(lidar_iris  SHARED
    LidarIris.cpp
    fftm/fftm.cpp
)
target_link_libraries(lidar_iris
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBS}
)

add_executable(demo demo.cpp )
target_link_libraries(demo lidar_iris)

