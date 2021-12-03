cmake_minimum_required(VERSION 2.8.3)
project(linefit_ground_segmentation)

add_definitions(-std=c++11 -g)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV 3 REQUIRED)

#set(Eigen3_DIR /usr/local/eigen-3.3.7/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 include: ${EIGEN3_INCLUDE_DIRS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)



#######################
#       BUILD
######################

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
	src/ground_segmentation.cc 
	src/segment.cc 
	src/bin.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
)



add_executable(${PROJECT_NAME}_test
    examples/test_linefit_ground_extractor.cpp
)
target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME} 
)