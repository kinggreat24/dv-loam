/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <d2vl_slam/dynamic_reconfigureConfig.h>

#include "d2vl_slam/SaveMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "System.h"
#include "Converter.h"

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



class Node
{
  public:
    Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();
    void Init ();
    void SaveGroundTruthTrajectory(const std::string& filename);
  protected:
    void Update ();
    void Update (const int frame_id, const bool isKeyFrame);
    ORB_SLAM2::System* orb_slam_;
    ros::Time current_frame_time_;

    std::string camera_info_topic_;

    //相机真实轨迹
    std::vector<Eigen::Matrix4d > mv_gt_poses_;
  private:
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    void PublisherMarker(cv::Mat& Tcw, const int id,const bool isKeyFrame);
    void PublishGroundTruthPose(Eigen::Matrix4d gt_pose_twc);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishGBAStatus (bool gba_status);
    void PublishRenderedImage (cv::Mat image);
    void ParamsChangedCallback(d2vl_slam::dynamic_reconfigureConfig &config, uint32_t level);
    bool SaveMapSrv (d2vl_slam::SaveMap::Request &req, d2vl_slam::SaveMap::Response &res);
    void LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters);
    void LoadOrbParameters (std::string& setting_file, ORB_SLAM2::ORBParameters& parameters);
    void LoadGroundTruthPose(const std::string& gt_file, std::vector<Eigen::Matrix4d >& gtPoses, bool convert_to_origin = false);

    // initialization Transform listener
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    tf2::Transform TransformFromMat (cv::Mat position_mat);
    tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);

    dynamic_reconfigure::Server<d2vl_slam::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher status_gba_publisher_;

    ros::Publisher local_lidar_map_points_publisher_;


    nav_msgs::Path lidar_odom_path_, gt_path_;
    ros::Publisher path_publisher_, gt_path_publisher_;

    visualization_msgs::MarkerArray pose_markers_;
    ros::Publisher pose_marker_publisher_;

    ros::ServiceServer service_server_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ORB_SLAM2::System::eSensor sensor_;

    std::string setting_file_param_;
    std::string gt_pose_file_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    int min_observations_per_point_;
};

#endif //ORBSLAM2_ROS_NODE_H_
