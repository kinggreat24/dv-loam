/*
 * @Author: kinggreat24
 * @Date: 2021-03-31 19:23:35
 * @LastEditTime: 2021-03-31 22:21:22
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/ros/include/VisualLidarNode.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
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

#ifndef ORBSLAM2_ROS_VISUAL_LIDAR_NODE_H_
#define ORBSLAM2_ROS_VISUAL_LIDAR_NODE_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include "System.h"
#include "Node.h"


class VisualLidarNode : public Node
{
  public:
    VisualLidarNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~VisualLidarNode ();
    void ImageLidarCallback (const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::PointCloud2ConstPtr &msgL);
    void start();
  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_subscriber_;
    message_filters::Synchronizer<sync_pol> *sync_;
};

#endif //ORBSLAM2_ROS_VISUAL_LIDAR_NODE_H_
