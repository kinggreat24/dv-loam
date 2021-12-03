/*
 * @Author: your name
 * @Date: 2020-04-09 10:36:44
 * @LastEditTime: 2020-05-21 19:48:32
 * @LastEditors: Kinggreat24
 * @Description: In User Settings Edit
 * @FilePath: /ORB_SLAM2-master/include/Common.h
 */
// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#ifndef ORB_SLAM2_COMMON_H
#define ORB_SLAM2_COMMON_H

#include <chrono>
#include <vector>

//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>            //条件滤波
#include <pcl/filters/voxel_grid.h>                     //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>    //统计滤波
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>                      //NDT(正态分布)配准类头文件

//OpenCV
#include <opencv2/opencv.hpp>

namespace loam {

using Time = std::chrono::system_clock::time_point;

////////////////////////////////////////////////////////////////////////
//                            Lego-LOAM TYPES
/////////////////////////////////////////////////////////////////////////
typedef struct Cloud_info{
    std::vector<int> startRingIndex;
    std::vector<int> endRingIndex;
    float startOrientation;
    float endOrientation;
    float orientationDiff;
    std::vector<bool> segmentedCloudGroundFlag;
    std::vector<unsigned int> segmentedCloudColInd;
    std::vector<float> segmentedCloudRange;
}Cloud_info;


typedef struct LidarParam{
    int   N_SCAN;                      //激光线数
    int   Horizon_SCAN;                //激光每一个ring上的激光点数
    float horizontalResolution;        //激光点水平分辨率
    float verticalResolution;          //激光点水平分辨率
    float topAngle;                    //激光雷达最上面的垂直角度
    float baseAngle;                   //激光雷达最下面的垂直角度
    std::vector<float> ringAngles;     //激光每个ring的俯仰角度

    //点云分割参数
    int   groundScanInd;              //地面点开始的ring
    float sensorMountAngle;           //地面点起伏阈值
    float segmentTheta;               //点云分割时的角度跨度上限（π/3）
    float segmentAlphaX;              //点云图像水平分辨率
    float segmentAlphaY;              //点云图像垂直方向分辨
    int   segmentValidPointNum;       //最小点簇的个数
    int   segmentValidLineNum;        //最小线簇的个数
    int   feasibleSegment;            //可见点簇的点云个数

    float scanPeriod;                 //激光的频率

    float bvXrange;                   //鸟瞰图x方向最大距离
    float bvYrange;                   //鸟瞰图y方向最大距离
    float bvResolution;               //鸟瞰图分辨率
}LidarParam;

typedef pcl::PointXYZI PointType;

typedef pcl::PointXYZRGB PointColor;

// typedef pcl::PointXYZRGBL PointColor;

typedef struct smoothness_t{ 
    float value;
    size_t ind;
}smoothness_t;

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) 
    { 
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
// struct PointXYZIL
// {
//     PCL_ADD_POINT4D
//     PCL_ADD_INTENSITY;
//     int32_t label;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIL,
//                                    (float, x, x) (float, y, y)
//                                    (float, z, z) (float, intensity, intensity)
//                                    (int32_t, label, label)
// )

// typedef PointXYZIL  PointTypeLabel;

//将上一帧的点投影到当前帧
inline void TransformToEnd(const PointType* point_last,const Eigen::Quaterniond& q_cur_last, const Eigen::Vector3d& t_cur_last, PointType* point_cur)
{
    Eigen::Vector3d pl(point_last->x,point_last->y,point_last->z);
    Eigen::Vector3d pc = q_cur_last.toRotationMatrix() * pl + t_cur_last;
    point_cur->x = pc[0];
    point_cur->y = pc[1];
    point_cur->z = pc[2];
    point_cur->intensity = point_last->intensity;
}

//将当前帧的点投影到上一帧
inline void TransformToStart(const PointType& point_cur,const Eigen::Quaterniond& q_last_cur, const Eigen::Vector3d& t_last_cur, PointType& point_last)
{
    Eigen::Vector3d pc(point_cur.x,point_cur.y,point_cur.z);
    Eigen::Vector3d pl = q_last_cur.toRotationMatrix() * pc + t_last_cur;
    point_last.x = pl[0];
    point_last.y = pl[1];
    point_last.z = pl[2];
    point_last.intensity = point_cur.intensity;
}

inline void TransformPoint(const PointType* point_cam, const Eigen::Quaterniond& q_wc, const Eigen::Vector3d& t_wc, PointType* point_world)
{
    Eigen::Vector3d pc(point_cam->x,point_cam->y,point_cam->z);
    Eigen::Vector3d pw = q_wc.toRotationMatrix() * pc + t_wc;
    point_world->x = pw[0];
    point_world->y = pw[1];
    point_world->z = pw[2];
    point_world->intensity = point_cam->intensity;
}


int TransformPointCloud(const pcl::PointCloud<PointType>::Ptr& pointcloud_in, const Eigen::Quaterniond& q, const Eigen::Vector3d& t,pcl::PointCloud<PointType>::Ptr& pointcloud_out);
int TransformPointCloud(const pcl::PointCloud<PointColor>::Ptr& pointcloud_in, const Eigen::Quaterniond& q, const Eigen::Vector3d& t,pcl::PointCloud<PointColor>::Ptr& pointcloud_out);

int DepthFilterPointCloud(const pcl::PointCloud<PointType>::Ptr& pointcloud_in,const char* field, const float min, const float max,pcl::PointCloud<PointType>::Ptr& pointcloud_out);
int DepthFilterPointCloud(const pcl::PointCloud<PointColor>::Ptr& pointcloud_in,const char* field, const float min, const float max,pcl::PointCloud<PointColor>::Ptr& pointcloud_out);


void SavePointCloudPly(const std::string& file_name, const pcl::PointCloud<PointType>::Ptr pc);
void SavePointCloudPly(const std::string& file_name, const pcl::PointCloud<PointColor>::Ptr pc);

/** \brief A standard non-ROS alternative to ros::Time.*/
// helper function
inline double toSec(Time::duration duration)
{
  return std::chrono::duration<double>(duration).count();
}

inline int64_t timestamp_now()
{
    std::chrono::system_clock::time_point now = std::chrono::high_resolution_clock::now();
    std::chrono::system_clock::duration duration_now = now.time_since_epoch();

    return std::chrono::duration_cast<std::chrono::microseconds>(duration_now).count();
}


//计算内点阈值
double compute_inlier_residual_threshold( const std::vector<double>& residuals, const int residual_size, const float m_inlier_ratio );




} // end namespace loam

#endif // ORB_SLAM2_COMMON_H
