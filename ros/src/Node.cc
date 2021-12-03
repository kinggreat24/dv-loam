#include "Node.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

Node::Node(ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : image_transport_(image_transport)
{
	name_of_node_ = ros::this_node::getName();
	node_handle_ = node_handle;
	min_observations_per_point_ = 2;
	sensor_ = sensor;
}

Node::~Node()
{
	// Stop all threads
	orb_slam_->Shutdown();

	// Save camera trajectory
	// orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	delete orb_slam_;
}

void Node::Init()
{
	//static parameters
	node_handle_.param(name_of_node_ + "/publish_pointcloud", publish_pointcloud_param_, true);
	node_handle_.param(name_of_node_ + "/publish_pose", publish_pose_param_, true);
	node_handle_.param(name_of_node_ + "/publish_tf", publish_tf_param_, true);
	node_handle_.param<std::string>(name_of_node_ + "/pointcloud_frame_id", map_frame_id_param_, "map");
	node_handle_.param<std::string>(name_of_node_ + "/camera_frame_id", camera_frame_id_param_, "camera_link");
	node_handle_.param<std::string>(name_of_node_ + "/target_frame_id", target_frame_id_param_, "base_link");
	node_handle_.param<std::string>(name_of_node_ + "/map_file", map_file_name_param_, "map.bin");
	node_handle_.param<std::string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
	node_handle_.param(name_of_node_ + "/load_map", load_map_param_, false);

	node_handle_.param<std::string>(name_of_node_ + "/setting_file_param", setting_file_param_, "");
	node_handle_.param<std::string>(name_of_node_ + "/gt_pose_file", gt_pose_file_, "");

	//读取相机姿态真值
	if(!gt_pose_file_.empty())
		LoadGroundTruthPose(gt_pose_file_, mv_gt_poses_);

	// Create a parameters object to pass to the Tracking system
	ORB_SLAM2::ORBParameters parameters;
	if(setting_file_param_.empty())
		LoadOrbParameters(parameters);
	else
		LoadOrbParameters(setting_file_param_,parameters);

	orb_slam_ = new ORB_SLAM2::System(voc_file_name_param_, sensor_, parameters, map_file_name_param_, load_map_param_);

	service_server_ = node_handle_.advertiseService(name_of_node_ + "/save_map", &Node::SaveMapSrv, this);

	//Setup dynamic reconfigure
	dynamic_reconfigure::Server<d2vl_slam::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
	dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
	dynamic_param_server_.setCallback(dynamic_param_callback);

	// Initialization transformation listener
	tfBuffer.reset(new tf2_ros::Buffer);
	tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

	rendered_image_publisher_ = image_transport_.advertise(name_of_node_ + "/debug_image", 1);
	if (publish_pointcloud_param_)
	{
		map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(name_of_node_ + "/map_points", 1);

		local_lidar_map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(name_of_node_ + "/local_lidar_map_points", 1);
	}

	// Enable publishing camera's pose as PoseStamped message
	if (publish_pose_param_)
	{
		pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(name_of_node_ + "/pose", 1);
		
		path_publisher_ = node_handle_.advertise<nav_msgs::Path>(name_of_node_ + "/path", 1);
		lidar_odom_path_.header.frame_id = map_frame_id_param_;

		pose_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(name_of_node_ + "/frame_marker", 1);


		//真值
		gt_path_publisher_= node_handle_.advertise<nav_msgs::Path>(name_of_node_ + "/gt_path", 1);
		gt_path_.header.frame_id = map_frame_id_param_;
	}

	status_gba_publisher_ = node_handle_.advertise<std_msgs::Bool>(name_of_node_ + "/gba_running", 1);
}

void Node::SaveGroundTruthTrajectory(const std::string& filename)
{
	if(mv_gt_poses_.size() <0)
		return;

	std::ofstream f;
	f.open(filename.c_str());
	f << fixed;

	for(size_t i=0;i<mv_gt_poses_.size();i++)
	{
		Eigen::Matrix4d gt_pose_twc = mv_gt_poses_.at(i);

		static Eigen::Matrix4d init_origin_Twc = gt_pose_twc;
		gt_pose_twc = init_origin_Twc.inverse() * gt_pose_twc;

		Eigen::Matrix3d Rwc = gt_pose_twc.block<3,3>(0,0);
		Eigen::Vector3d twc = gt_pose_twc.block<3,1>(0,3);

		f 	<< setprecision(9) 
			<< Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0) << " " 
			<< Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " " 
			<< Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
	}
	f.flush();
	f.close();
	std::cout << std::endl << "trajectory saved!" << std::endl;
}



void Node::Update()
{
	cv::Mat position = orb_slam_->GetCurrentPosition();

	if (!position.empty())
	{
		if (publish_tf_param_)
		{
			PublishPositionAsTransform(position);
		}

		if (publish_pose_param_)
		{
			PublishPositionAsPoseStamped(position);
		}
	}

	PublishRenderedImage(orb_slam_->DrawCurrentFrame());

	if (publish_pointcloud_param_)
	{
		PublishMapPoints(orb_slam_->GetAllMapPoints());
	}

	PublishGBAStatus(orb_slam_->isRunningGBA());
}


void Node::Update (const int frame_id, const bool isKeyFrame)
{
	cv::Mat position = orb_slam_->GetCurrentPosition();

	if (!position.empty())
	{
		if (publish_tf_param_)
		{
			PublishPositionAsTransform(position);
		}

		if (publish_pose_param_)
		{
			PublishPositionAsPoseStamped(position);
		}
	}

	PublishRenderedImage(orb_slam_->DrawCurrentFrame());

	// 发布特征点地图
	// if (publish_pointcloud_param_)
	// {
	// 	PublishMapPoints(orb_slam_->GetAllMapPoints());
	// }

	PublishGBAStatus(orb_slam_->isRunningGBA());

	//发布真值轨迹
	if(mv_gt_poses_.size() > 0)
		PublishGroundTruthPose(mv_gt_poses_[frame_id]);

	//发布当前帧的位姿marker
	PublisherMarker(position, frame_id, isKeyFrame);


	if(sensor_ == ORB_SLAM2::System::VISUAL_LIDAR)
	{
		sensor_msgs::PointCloud2 PointsMsg;
		pcl::PointCloud<pcl::PointXYZI>::Ptr local_pc_cloud(new pcl::PointCloud<pcl::PointXYZI>());
		if(!orb_slam_->GetTracker()->mp_sw_optimizater)
			return;
		orb_slam_->GetTracker()->mp_sw_optimizater->getMap(local_pc_cloud);
		if(local_pc_cloud->size() <=0)
			return;
		Eigen::Matrix4d camera2world = Eigen::Matrix4d::Identity();
		camera2world <<  0,  0, 1, 0,
						-1,  0, 0, 0,
						 0, -1, 0, 0,
						 0,  0, 0, 1;
		pcl::PointCloud<pcl::PointXYZI>::Ptr local_world_pc_map(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*(local_pc_cloud), *local_world_pc_map, camera2world);
		pcl::toROSMsg(*local_world_pc_map, PointsMsg);
		PointsMsg.header.stamp = current_frame_time_;
		PointsMsg.header.frame_id = "map";

		//发布局部地图
		local_lidar_map_points_publisher_.publish(PointsMsg);
	}
}


void Node::PublishMapPoints(std::vector<ORB_SLAM2::MapPoint *> map_points)
{
	sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud(map_points);
	map_points_publisher_.publish(cloud);
}

void Node::PublishGroundTruthPose(Eigen::Matrix4d gt_pose_twc)
{
	static Eigen::Matrix4d init_origin_Twc = gt_pose_twc;

	gt_pose_twc = init_origin_Twc.inverse() * gt_pose_twc;

	Eigen::Matrix3d R_transform;
	R_transform << 	 0,  0, 1,
					-1,  0, 0,
					 0, -1, 0;
    Eigen::Vector3d translation =  R_transform * gt_pose_twc.block<3,1>(0,3);

	Eigen::Quaterniond q_w_i(gt_pose_twc.topLeftCorner<3, 3>());
    Eigen::Quaterniond q = Eigen::Quaterniond(R_transform) * q_w_i;
    q.normalize();

	geometry_msgs::PoseStamped posestamped_gt;
	posestamped_gt.pose.position.x = translation.x();
	posestamped_gt.pose.position.y = translation.y();
	posestamped_gt.pose.position.z = translation.z();
	posestamped_gt.pose.orientation.x = q.x();
    posestamped_gt.pose.orientation.y = q.y();
    posestamped_gt.pose.orientation.z = q.z();
    posestamped_gt.pose.orientation.w = q.w();

	gt_path_.header.stamp=current_frame_time_;
    gt_path_.poses.push_back(posestamped_gt);
	gt_path_publisher_.publish(gt_path_);
}

void Node::PublisherMarker(cv::Mat& Tcw, const int id, const bool isKeyFrame)
{
    Eigen::Matrix4f twc = ORB_SLAM2::Converter::toMatrix4f(Tcw).inverse();
	Eigen::Matrix3f R_transform;
	R_transform << 	 0,  0, 1,
					-1,  0, 0,
					 0, -1, 0;
    Eigen::Vector3f translation =  R_transform * twc.block<3,1>(0,3);

    Eigen::Quaternionf q_w_i(twc.topLeftCorner<3, 3>());
    Eigen::Quaternionf q = Eigen::Quaternionf(R_transform) * q_w_i;
    q.normalize();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "my_namespace";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = current_frame_time_;
    marker.id = id;
    marker.pose.position.x = translation(0);
    marker.pose.position.y = translation(1);
    marker.pose.position.z = translation(2);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.a = 1.0;            // Don't forget to set the alpha!
	if(isKeyFrame)
	{	
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	else
	{
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}
    
    pose_marker_publisher_.publish(marker);
}


tf2::Transform Node::TransformToTarget(tf2::Transform tf_in, std::string frame_in, std::string frame_target)
{
	// Transform tf_in from frame_in to frame_target
	tf2::Transform tf_map2orig = tf_in;
	tf2::Transform tf_orig2target;
	tf2::Transform tf_map2target;

	tf2::Stamped<tf2::Transform> transformStamped_temp;
	try
	{
		// Get the transform from camera to target
		geometry_msgs::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, ros::Time(0));
		// Convert to tf2
		tf2::fromMsg(tf_msg, transformStamped_temp);
		tf_orig2target.setBasis(transformStamped_temp.getBasis());
		tf_orig2target.setOrigin(transformStamped_temp.getOrigin());
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		//ros::Duration(1.0).sleep();
		tf_orig2target.setIdentity();
	}

	// Transform from map to target
	tf_map2target = tf_map2orig * tf_orig2target;
	return tf_map2target;
}

void Node::PublishPositionAsTransform(cv::Mat position)
{
	// Get transform from map to camera frame
	tf2::Transform tf_transform = TransformFromMat(position);

	// Make transform from camera frame to target frame
	tf2::Transform tf_map2target = TransformToTarget(tf_transform, camera_frame_id_param_, target_frame_id_param_);

	// Make message
	tf2::Stamped<tf2::Transform> tf_map2target_stamped;
	tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, current_frame_time_, map_frame_id_param_);
	geometry_msgs::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
	msg.child_frame_id = target_frame_id_param_;
	// Broadcast tf
	static tf2_ros::TransformBroadcaster tf_broadcaster;
	tf_broadcaster.sendTransform(msg);
}

void Node::PublishPositionAsPoseStamped(cv::Mat position)
{
	tf2::Transform tf_position = TransformFromMat(position);

	// Make transform from camera frame to target frame
	tf2::Transform tf_position_target = TransformToTarget(tf_position, camera_frame_id_param_, target_frame_id_param_);

	// Make message
	tf2::Stamped<tf2::Transform> tf_position_target_stamped;
	tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position_target, current_frame_time_, map_frame_id_param_);
	geometry_msgs::PoseStamped pose_msg;
	tf2::toMsg(tf_position_target_stamped, pose_msg);
	pose_publisher_.publish(pose_msg);

	// 发布轨迹
	lidar_odom_path_.header.stamp=ros::Time::now();
    lidar_odom_path_.poses.push_back(pose_msg);
	path_publisher_.publish(lidar_odom_path_);
}

void Node::PublishGBAStatus(bool gba_status)
{
	std_msgs::Bool gba_status_msg;
	gba_status_msg.data = gba_status;
	status_gba_publisher_.publish(gba_status_msg);
}

void Node::PublishRenderedImage(cv::Mat image)
{
	std_msgs::Header header;
	header.stamp = current_frame_time_;
	header.frame_id = map_frame_id_param_;
	const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
	rendered_image_publisher_.publish(rendered_image_msg);
}

tf2::Transform Node::TransformFromMat(cv::Mat position_mat)
{
	cv::Mat rotation(3, 3, CV_32F);
	cv::Mat translation(3, 1, CV_32F);

	rotation = position_mat.rowRange(0, 3).colRange(0, 3);
	translation = position_mat.rowRange(0, 3).col(3);

	tf2::Matrix3x3 tf_camera_rotation(rotation.at<float>(0, 0), rotation.at<float>(0, 1), rotation.at<float>(0, 2),
									  rotation.at<float>(1, 0), rotation.at<float>(1, 1), rotation.at<float>(1, 2),
									  rotation.at<float>(2, 0), rotation.at<float>(2, 1), rotation.at<float>(2, 2));

	tf2::Vector3 tf_camera_translation(translation.at<float>(0), translation.at<float>(1), translation.at<float>(2));

	//Coordinate transformation matrix from orb coordinate system to ros coordinate system
	const tf2::Matrix3x3 tf_orb_to_ros(0, 0, 1,
									   -1, 0, 0,
									   0, -1, 0);

	//Transform from orb coordinate system to ros coordinate system on camera coordinates
	tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
	tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

	//Inverse matrix
	tf_camera_rotation = tf_camera_rotation.transpose();
	tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

	//Transform from orb coordinate system to ros coordinate system on map coordinates
	tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
	tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

	return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

sensor_msgs::PointCloud2 Node::MapPointsToPointCloud(std::vector<ORB_SLAM2::MapPoint *> map_points)
{
	if (map_points.size() == 0)
	{
		std::cout << "Map point vector is empty!" << std::endl;
		
	}

	sensor_msgs::PointCloud2 cloud;

	const int num_channels = 3; // x y z

	cloud.header.stamp    = current_frame_time_;
	cloud.header.frame_id = map_frame_id_param_;
	cloud.height = 1;
	cloud.width = map_points.size();
	cloud.is_bigendian = false;
	cloud.is_dense = true;
	cloud.point_step = num_channels * sizeof(float);
	cloud.row_step = cloud.point_step * cloud.width;
	cloud.fields.resize(num_channels);

	std::string channel_id[] = {"x", "y", "z"};
	for (int i = 0; i < num_channels; i++)
	{
		cloud.fields[i].name = channel_id[i];
		cloud.fields[i].offset = i * sizeof(float);
		cloud.fields[i].count = 1;
		cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
	}

	cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

	float data_array[num_channels];
	for (unsigned int i = 0; i < cloud.width; i++)
	{
		if (map_points.at(i)->nObs >= min_observations_per_point_)
		{
			data_array[0] = map_points.at(i)->GetWorldPos().at<float>(2);		 //x. Do the transformation by just reading at the position of z instead of x
			data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(0); //y. Do the transformation by just reading at the position of x instead of y
			data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(1); //z. Do the transformation by just reading at the position of y instead of z
			//TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

			memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
		}
	}

	return cloud;
}

void Node::ParamsChangedCallback(d2vl_slam::dynamic_reconfigureConfig &config, uint32_t level)
{
	orb_slam_->EnableLocalizationOnly(config.localize_only);
	min_observations_per_point_ = config.min_observations_for_ros_map;

	if (config.reset_map)
	{
		orb_slam_->Reset();
		config.reset_map = false;
	}

	orb_slam_->SetMinimumKeyFrames(config.min_num_kf_in_map);
}

bool Node::SaveMapSrv(d2vl_slam::SaveMap::Request &req, d2vl_slam::SaveMap::Response &res)
{
	res.success = orb_slam_->SaveMap(req.name);

	if (res.success)
	{
		ROS_INFO_STREAM("Map was saved as " << req.name);
	}
	else
	{
		ROS_ERROR("Map could not be saved.");
	}

	return res.success;
}

void Node::LoadOrbParameters(ORB_SLAM2::ORBParameters &parameters)
{
	//ORB SLAM configuration parameters
	node_handle_.param(name_of_node_ + "/camera_fps", parameters.maxFrames, 30);
	node_handle_.param(name_of_node_ + "/camera_rgb_encoding", parameters.RGB, true);
	node_handle_.param(name_of_node_ + "/ORBextractor/nFeatures", parameters.nFeatures, 1200);
	node_handle_.param(name_of_node_ + "/ORBextractor/scaleFactor", parameters.scaleFactor, static_cast<float>(1.2));
	node_handle_.param(name_of_node_ + "/ORBextractor/nLevels", parameters.nLevels, 8);
	node_handle_.param(name_of_node_ + "/ORBextractor/iniThFAST", parameters.iniThFAST, 20);
	node_handle_.param(name_of_node_ + "/ORBextractor/minThFAST", parameters.minThFAST, 7);

	bool load_calibration_from_cam = false;
	node_handle_.param(name_of_node_ + "/load_calibration_from_cam", load_calibration_from_cam, false);

	if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD)
	{
		node_handle_.param(name_of_node_ + "/ThDepth", parameters.thDepth, static_cast<float>(35.0));
		node_handle_.param(name_of_node_ + "/depth_map_factor", parameters.depthMapFactor, static_cast<float>(1.0));
	}

	if (load_calibration_from_cam)
	{
		ROS_INFO_STREAM("Listening for camera info on topic " << node_handle_.resolveName(camera_info_topic_));
		sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, ros::Duration(1000.0));
		if (camera_info == nullptr)
		{
			ROS_WARN("Did not receive camera info before timeout, defaulting to launch file params.");
		}
		else
		{
			parameters.fx = camera_info->K[0];
			parameters.fy = camera_info->K[4];
			parameters.cx = camera_info->K[2];
			parameters.cy = camera_info->K[5];

			parameters.baseline = camera_info->P[3];

			parameters.k1 = camera_info->D[0];
			parameters.k2 = camera_info->D[1];
			parameters.p1 = camera_info->D[2];
			parameters.p2 = camera_info->D[3];
			parameters.k3 = camera_info->D[4];
			return;
		}
	}

	bool got_cam_calibration = true;
	if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD)
	{
		got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_baseline", parameters.baseline);
	}
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_width",  parameters.width);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_height", parameters.height);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fx", parameters.fx);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fy", parameters.fy);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cx", parameters.cx);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cy", parameters.cy);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k1", parameters.k1);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k2", parameters.k2);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p1", parameters.p1);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p2", parameters.p2);
	got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k3", parameters.k3);

	if (!got_cam_calibration)
	{
		ROS_ERROR("Failed to get camera calibration parameters from the launch file.");
		throw std::runtime_error("No cam calibration");
	}
}

void Node::LoadOrbParameters (std::string& setting_file, ORB_SLAM2::ORBParameters& parameters)
{
	//Check settings file
    cv::FileStorage fsSettings(setting_file.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << setting_file << endl;
       exit(-1);
    }

	parameters.maxFrames = fsSettings["Camera.fps"];
	int rgb_encoding     = fsSettings["Camera.RGB"];
	if(rgb_encoding)
		parameters.RGB = true;
	else
		parameters.RGB = false;

	// read orb feature params
	parameters.nFeatures = fsSettings["ORBextractor.nFeatures"];
	parameters.scaleFactor = fsSettings["ORBextractor.scaleFactor"];
	parameters.nLevels = fsSettings["ORBextractor.nLevels"];
	parameters.iniThFAST = fsSettings["ORBextractor.iniThFAST"];
	parameters.minThFAST = fsSettings["ORBextractor.minThFAST"];

	// read camera instrics
	parameters.width  = fsSettings["Camera.width"];
	parameters.height = fsSettings["Camera.height"];
	parameters.fx = fsSettings["Camera.fx"];
	parameters.fy = fsSettings["Camera.fy"];
	parameters.cx = fsSettings["Camera.cx"];
	parameters.cy = fsSettings["Camera.cy"];
	parameters.k1 = fsSettings["Camera.k1"];
	parameters.k2 = fsSettings["Camera.k2"];
	parameters.p1 = fsSettings["Camera.p1"];
	parameters.p2 = fsSettings["Camera.p2"];
	parameters.k3 = fsSettings["Camera.k3"];

	if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD || sensor_ == ORB_SLAM2::System::VISUAL_LIDAR)
	{
		parameters.thDepth = fsSettings["ThDepth"];
		parameters.depthMapFactor = fsSettings["DepthMapFactor"];
	}

	if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD)
	{
		parameters.baseline = fsSettings["Camera.bf"];
	}

	// 相机与激光雷达的外参
	int use_lidar = fsSettings["use_lidar"];
	if(use_lidar != 0)
	{
		std::cout<<"**********************     use lidar      ****************************"<<std::endl;
		parameters.use_lidar = true;

		cv::Mat T;
		int need_inverse =  fsSettings["extrinsicMatrix.need_inverse"];
		fsSettings["extrinsicMatrix"] >> T;
		cv::cv2eigen(T, parameters.cameraLidarExtrinsic);
		if(need_inverse)
		{
			Eigen::Matrix3f R = parameters.cameraLidarExtrinsic.topLeftCorner(3,3);
			Eigen::Vector3f t = parameters.cameraLidarExtrinsic.topRightCorner(3,1);

			parameters.cameraLidarExtrinsic.topLeftCorner(3,3)  =  R.transpose();
			parameters.cameraLidarExtrinsic.topRightCorner(3,1) = -R.transpose()*t;
		}

		parameters.lidar_name = string(fsSettings["Lidar_Name"]);

		parameters.groundSegmentationParams.n_threads         = fsSettings["LineFitGroundSegmentation.n_threads"];
		float r_min = fsSettings["LineFitGroundSegmentation.r_min"];
		parameters.groundSegmentationParams.r_min_square      = r_min * r_min;
		float r_max = fsSettings["LineFitGroundSegmentation.r_max"];
		parameters.groundSegmentationParams.r_max_square      = r_max * r_max;
		parameters.groundSegmentationParams.n_bins            = fsSettings["LineFitGroundSegmentation.n_bins"];       
		parameters.groundSegmentationParams.n_segments        = fsSettings["LineFitGroundSegmentation.n_segments"];
		parameters.groundSegmentationParams.max_dist_to_line  = fsSettings["LineFitGroundSegmentation.max_dist_to_line"];
		parameters.groundSegmentationParams.sensor_height     = fsSettings["LineFitGroundSegmentation.sensor_height"];
		parameters.groundSegmentationParams.max_slope         = fsSettings["LineFitGroundSegmentation.max_slope"];
		float max_fit_error = fsSettings["LineFitGroundSegmentation.max_fit_error"];
		parameters.groundSegmentationParams.max_error_square  = max_fit_error * max_fit_error;
		parameters.groundSegmentationParams.long_threshold    = fsSettings["LineFitGroundSegmentation.long_threshold"];
		parameters.groundSegmentationParams.max_long_height   = fsSettings["LineFitGroundSegmentation.max_long_height"];
		parameters.groundSegmentationParams.max_start_height  = fsSettings["LineFitGroundSegmentation.max_start_height"];
		parameters.groundSegmentationParams.line_search_angle = fsSettings["LineFitGroundSegmentation.line_search_angle"];
		parameters.groundSegmentationParams.kitti_camera_cooridinate = false;
		parameters.groundSegmentationParams.visualize = false;

		// 直接法跟踪
		parameters.tracker_.levels          = fsSettings["Tracker.levels"];
		parameters.tracker_.min_level       = fsSettings["Tracker.min_level"];
		parameters.tracker_.max_level       = fsSettings["Tracker.max_level"];
		parameters.tracker_.max_iteration   = fsSettings["Tracker.max_iteration"];
		parameters.tracker_.scale_estimator = string(fsSettings["Tracker.scale_estimator"]);
		parameters.tracker_.weight_function = string(fsSettings["Tracker.weight_function"]);
		parameters.tracker_.set_scale_estimator_type();
		parameters.tracker_.set_weight_function_type();


		//激光雷达闭环检测
		parameters.iris_nscale        = fsSettings["LidarIris.nscale"];
		parameters.iris_minWaveLength = fsSettings["LidarIris.minWaveLength"];
		parameters.iris_mult          = fsSettings["LidarIris.mult"];
		parameters.iris_sigmaOnf      = fsSettings["LidarIris.sigmaOnf"];
		parameters.iris_matchNum      = fsSettings["LidarIris.matchNum"];
	}
	else
		parameters.use_lidar = false;
}


void Node::LoadGroundTruthPose(const std::string& gt_file, std::vector<Eigen::Matrix4d >& gtPoses, bool convert_to_origin)
{
    FILE *fp = fopen(gt_file.c_str(),"r");
    if (!fp)
        return;

    Eigen::Matrix4d pose_origin = Eigen::Matrix4d::Identity();
    while (!feof(fp)) 
    {
        Eigen::Matrix<double,3,4> P;
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    &P(0,0), &P(0,1), &P(0,2), &P(0,3),
                    &P(1,0), &P(1,1), &P(1,2), &P(1,3),
                    &P(2,0), &P(2,1), &P(2,2), &P(2,3) )==12) 
        {
            if(convert_to_origin)
            {   
                if(gtPoses.size()>0)
                {
                    //转换到以第一帧相机为原点的坐标系中
                    Eigen::Matrix4d raw_pose = Eigen::Matrix4d::Identity();
                    raw_pose.block<3,4>(0,0) = P;
                    Eigen::Matrix4d converted_pose = pose_origin.inverse() * raw_pose;
                    gtPoses.push_back(converted_pose);
                }
                else
                {
                    gtPoses.push_back(pose_origin);
                
                    //第一帧姿态
                    pose_origin.block<3,4>(0,0) = P;
                }
            }
            else
            {
				Eigen::Matrix4d cur_pose = Eigen::Matrix4d::Identity();
				cur_pose.block<3,4>(0,0) = P;
				gtPoses.push_back(cur_pose);
			}    
        }
    }
    fclose(fp);
}
