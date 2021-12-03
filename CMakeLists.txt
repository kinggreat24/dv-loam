<<<<<<< HEAD
cmake_minimum_required(VERSION 2.8.3)
project(d2vl_slam)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

## Check C++11 or C++0x support
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
message(STATUS "Using flag -std=c++14.")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/orb_slam2/cmake_modules)

find_package (catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
cv_bridge
image_transport
tf
sensor_msgs
nav_msgs
visualization_msgs
dynamic_reconfigure
message_generation
)

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
     endif()
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


##Sophus
#find_package(Sophus REQUIRED)
set(Sophus_INCLUDE_DIRS /usr/local/Sophus/include)
set(Sophus_LIBRARIES /usr/local/Sophus/lib/libSophus.so)

##VIKIT_COMMON
set(vikit_common ${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/vikit_common/build)
find_package(vikit_common REQUIRED)

##PCL
find_package(PCL 1.7 REQUIRED QUIET)
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_FOUND)

## CERES
set(Ceres_DIR /usr/local/ceres-1.14/lib/cmake/Ceres)
find_package(Ceres REQUIRED)
message(STATUS "Ceres include : ${CERES_INCLUDE_DIRS}")

set (DYNAMIC_RECONFIGURE_PATH ros/config/dynamic_reconfigure.cfg)
execute_process(COMMAND chmod a+x ${DYNAMIC_RECONFIGURE_PATH}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        RESULT_VARIABLE cmd_result
        OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")

generate_dynamic_reconfigure_options(
        ${DYNAMIC_RECONFIGURE_PATH}
)

set(LIBS_ORBSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/lib/libg2o.so
)

set(LIBS_ROS
${PROJECT_SOURCE_DIR}/orb_slam2/lib/lib${PROJECT_NAME}.so
${OpenCV_LIBS}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/fadeRelease_v1.82/lib_ubuntu18.04_x86_64/libfade25d.so
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/LidarIris/lib/liblidar_iris.so
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/fast_gicp/lib/libfast_vgicp.so
${catkin_LIBRARIES}
)

add_service_files(
        FILES
        SaveMap.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

include_directories(
${PROJECT_SOURCE_DIR}/orb_slam2/include/loam
${PROJECT_SOURCE_DIR}/orb_slam2/include/depth_clustering
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/fadeRelease_v1.82
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/LidarIris
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/fast_gicp/include
${PROJECT_SOURCE_DIR}/orb_slam2/include
${PROJECT_SOURCE_DIR}/orb_slam2
${PROJECT_SOURCE_DIR}/ros/include
${EIGEN3_INCLUDE_DIR}
${Sophus_INCLUDE_DIRS}
${vikit_common_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam2/lib)

add_library(${PROJECT_NAME} SHARED
orb_slam2/src/System.cc
orb_slam2/src/Tracking.cc
orb_slam2/src/LocalMapping.cc
orb_slam2/src/LoopClosing.cc
orb_slam2/src/ORBextractor.cc
orb_slam2/src/ORBmatcher.cc
orb_slam2/src/FrameDrawer.cc
orb_slam2/src/Converter.cc
orb_slam2/src/MapPoint.cc
orb_slam2/src/KeyFrame.cc
orb_slam2/src/Map.cc
orb_slam2/src/Optimizer.cc
orb_slam2/src/PnPsolver.cc
orb_slam2/src/Frame.cc
orb_slam2/src/KeyFrameDatabase.cc
orb_slam2/src/Sim3Solver.cc
orb_slam2/src/Initializer.cc

orb_slam2/src/BalmClass.cc

## visual-lidar odometry
orb_slam2/src/lidar_sparse_align/SparseLidarAlign.cc
orb_slam2/src/lidar_sparse_align/SlideWindowSparseAlign.cc
orb_slam2/src/lidar_sparse_align/SemiDirectLidarAlign.cc
orb_slam2/src/lidar_sparse_align/WeightFunction.cpp


## linefit ground extraction
orb_slam2/src/ground_segmentation/ground_segmentation.cc 
orb_slam2/src/ground_segmentation/segment.cc 
orb_slam2/src/ground_segmentation/bin.cc

## Lidar feature extractor
orb_slam2/src/loam/BasicScanRegistration.cc
orb_slam2/src/loam/MultiScanRegistration.cc

## Lidar segmentation
orb_slam2/src/depth_clustering/utils/bbox.cc
orb_slam2/src/depth_clustering/utils/cloud.cc
orb_slam2/src/depth_clustering/utils/folder_reader.cc
orb_slam2/src/depth_clustering/utils/rich_point.cc
orb_slam2/src/depth_clustering/utils/velodyne_utils.cc
orb_slam2/src/depth_clustering/projections/projection_params.cc
orb_slam2/src/depth_clustering/projections/cloud_projection.cc
orb_slam2/src/depth_clustering/projections/ring_projection.cc
orb_slam2/src/depth_clustering/projections/spherical_projection.cc
orb_slam2/src/depth_clustering/ground_removal/depth_ground_remover.cpp
orb_slam2/src/depth_clustering/image_labelers/abstract_image_labeler.cpp
orb_slam2/src/depth_clustering/image_labelers/diff_helpers/angle_diff.cpp
orb_slam2/src/depth_clustering/image_labelers/diff_helpers/line_dist_diff.cpp


## Lidar LocalMapping
orb_slam2/src/CeresOptimizer.cc
orb_slam2/src/odomEstimationClass.cpp

)
add_dependencies (${PROJECT_NAME} g2o DBoW2)

target_link_libraries(${PROJECT_NAME}
${LIBS_ORBSLAM}
${Sophus_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES}
${vikit_common_LIBRARIES}
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/fadeRelease_v1.82/lib_ubuntu18.04_x86_64/libfade25d.so
#${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/linefit_ground_segmentation/lib/liblinefit_ground_segmentation.so
-lpthread
)

# map serialization addition - library boost serialization 
message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

add_executable (${PROJECT_NAME}_mono
ros/src/MonoNode.cc
ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_mono ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_mono
${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_stereo
ros/src/StereoNode.cc
ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_stereo ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_stereo
${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_rgbd
ros/src/RGBDNode.cc
ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_rgbd ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_rgbd
${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_visual_lidar
ros/src/VisualLidarNode.cc
ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_visual_lidar ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_visual_lidar
${LIBS_ROS}
)




install(TARGETS ${PROJECT_NAME}_mono ${PROJECT_NAME}_stereo ${PROJECT_NAME}_rgbd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} DBoW2 g2o
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(DIRECTORY orb_slam2/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/orb_slam2/config
)

install(DIRECTORY orb_slam2/Vocabulary/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/orb_slam2/Vocabulary
)