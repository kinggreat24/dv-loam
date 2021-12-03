/*
 * @Author: kinggreat24
 * @Date: 2021-03-31 17:08:58
 * @LastEditTime: 2021-08-23 15:02:56
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/ros/src/VisualLidarNode.cc
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "VisualLidarNode.h"

#include <vector>
#include <string>

using namespace std;

int lidar_type = 0;

std::string path_to_sequence;
std::string path_to_save_trajectory;
std::string path_to_save_gt_trajectory;
std::string path_to_save_time_;
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageGray,
        vector<string> &vstrImageLabel, vector<string> &vstrLidarFile, vector<double> &vTimestamps);

int end_frame_id_ = 9999;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MonoLidar");
	ros::start();

	if (argc > 1)
	{
		ROS_WARN("Arguments supplied via command line are neglected.");
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ros::NodeHandle node_handle;
	image_transport::ImageTransport image_transport(node_handle);
	VisualLidarNode node(ORB_SLAM2::System::VISUAL_LIDAR, node_handle, image_transport);
	node.Init();

    node.start();
	
    ros::shutdown();

	return 0;
}

VisualLidarNode::VisualLidarNode(ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node(sensor, node_handle, image_transport)
{
    rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/image_left", 1);
	lidar_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle, "/velodyne_points", 1);
    camera_info_topic_ = "/camera/camera_info";
    
	sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *rgb_subscriber_, *lidar_subscriber_);
	sync_->registerCallback(boost::bind(&VisualLidarNode::ImageLidarCallback, this, _1, _2));
}

VisualLidarNode::~VisualLidarNode()
{
    //保存轨迹
    // Save camera trajectory
	orb_slam_->SaveTrajectoryKITTI(path_to_save_trajectory);
    SaveGroundTruthTrajectory(path_to_save_gt_trajectory);

    //计算直接法的平均时间
    double total_time = 0.0;
    total_time = std::accumulate(orb_slam_->GetTracker()->mvTrackingTime.begin(), orb_slam_->GetTracker()->mvTrackingTime.end(),0.0);
    double mean_tracking_time = total_time*1.0f/orb_slam_->GetTracker()->mvTrackingTime.size();
    std::cout<<"Mean tracking time: "<<mean_tracking_time<<std::endl;

    // 窗口优化时间
    total_time = std::accumulate(orb_slam_->GetTracker()->mvSWTrackingTime.begin(), orb_slam_->GetTracker()->mvSWTrackingTime.end(),0.0);
    double mean_sw_opti_time = total_time*1.0f/orb_slam_->GetTracker()->mvSWTrackingTime.size();
    std::cout<<"Mean sw optimization time: "<<mean_sw_opti_time<<std::endl;
    
    
    total_time = std::accumulate(orb_slam_->GetTracker()->mvScan2MapTime.begin(), orb_slam_->GetTracker()->mvScan2MapTime.end(),0.0);
    double mean_sm_opti_time = total_time*1.0f/orb_slam_->GetTracker()->mvScan2MapTime.size();
    std::cout<<"Mean scan2map optimization time: "<<mean_sm_opti_time<<std::endl;
    
    //保存时间文件
    std::ofstream f;
	f.open(path_to_save_time_);
	f << fixed;
    f <<mean_tracking_time <<" "<<mean_sw_opti_time<<" "<< mean_sm_opti_time<<std::endl;
    f.flush();
    f.close();

    std::ofstream time_efficiency_info;
    time_efficiency_info.open("/home/kinggreat24/pc/time_efficiency_info.txt");
    std::vector<int64_t> keyframe_idx = orb_slam_->GetTracker()->mvScan2MapIndex;
    int cnt = 0;
    for(size_t i=0;i<orb_slam_->GetTracker()->mvSWTrackingTime.size();i++)
    {
        
        time_efficiency_info<<orb_slam_->GetTracker()->mvTrackingTime[i]<<" "<<orb_slam_->GetTracker()->mvSWTrackingTime[i];
        
        if(std::find(keyframe_idx.begin(),keyframe_idx.end(),i) == keyframe_idx.end())
        {
            time_efficiency_info <<std::endl;
        }    
        else
        {
            time_efficiency_info <<" "<<orb_slam_->GetTracker()->mvScan2MapTime[cnt++]<<std::endl;
        } 
    }
}

void VisualLidarNode::start()
{
    ros::NodeHandle nh_private("~");
    
    // Retrieve paths to images
    ROS_INFO("Load image data...");
    
    nh_private.param("lidar_type",lidar_type, 0);
    
    nh_private.param<std::string>("path_to_sequence",path_to_sequence,"00");
    nh_private.param<std::string>("path_to_save_trajectory",path_to_save_trajectory,"./pose_trjectory.txt");
    nh_private.param<std::string>("path_to_save_gt_trajectory",path_to_save_gt_trajectory,"./pose_gt_trjectory.txt");
    nh_private.param<std::string>("path_to_save_time",path_to_save_time_,"./time.txt");
    nh_private.param<int>("end_frame",end_frame_id_,end_frame_id_);
    vector<string> vstrImageGray;
    vector<string> vstrImageLabel;
    vector<string> vstrLidar;
    vector<double> vTimestamps;
    LoadImages(path_to_sequence, vstrImageGray, vstrImageLabel,vstrLidar, vTimestamps);
    ROS_INFO("Load %d images in the dataset.",(int)vstrImageGray.size());

    int nImages =static_cast<int>(vstrImageGray.size());
    nImages = std::min(nImages,end_frame_id_);
    
    ros::Rate rate(10);
    int ni = 0;
    cv::Mat imGray;
    while (ros::ok())
    {
        /* code */
        if(ni < nImages)
        {
            // read images
            ROS_INFO("********         Tracking frame %d       ***********",ni);
            imGray = cv::imread(vstrImageGray[ni], CV_LOAD_IMAGE_UNCHANGED);
            if(imGray.empty())
            {
                cerr << endl << "Failed to load image at: "
                    << string(vstrImageGray[ni]) << endl;
                return;
            }

	        orb_slam_->TrackMonocularLidar(imGray,vstrImageLabel[ni],vstrLidar[ni],vTimestamps[ni]);
            
            bool keyframe = false;
            if(orb_slam_->GetTracker()->GetCurrentKeyFrameId() == orb_slam_->GetTracker()->mCurrentFrame.mnId)
                keyframe = true;

            // Update();
            Update(orb_slam_->GetTracker()->mCurrentFrame.mnId, keyframe);

            ni++;
        }
        else
        {

        }
        
        ros::spinOnce();
        rate.sleep();
       
    }
}

//视觉激光雷达回调函数
void VisualLidarNode::ImageLidarCallback(const sensor_msgs::ImageConstPtr &msg,const sensor_msgs::PointCloud2ConstPtr &msgL)
{
	cv_bridge::CvImageConstPtr cv_in_ptr;
	try
	{
		cv_in_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    static int frame_cnt = 0;
    ROS_INFO("Frame id: %d",frame_cnt++);

	current_frame_time_ = msg->header.stamp;

	// orb_slam_->TrackMonocular(cv_in_ptr->image, cv_in_ptr->header.stamp.toSec());
	// Update();
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageGray,
                vector<string> &vstrImageLabel, vector<string> &vstrLidarFile, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixImg = strPathToSequence + "/image_0/";
    // string strPrefixLabel = strPathToSequence + "/objects/";
    string strPrefixLabel = strPathToSequence + "/dynamic_mask/";
    string strPrefixLidar = strPathToSequence + "/velodyne/";

    const int nTimes = vTimestamps.size();
    vstrImageGray.resize(nTimes);
    vstrImageLabel.resize(nTimes);
    vstrLidarFile.resize(nTimes);


    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageGray[i] = strPrefixImg + ss.str() + ".png";
        vstrImageLabel[i] = strPrefixLabel + ss.str() + ".jpg";
        
        if(lidar_type == 0)
            vstrLidarFile[i] = strPrefixLidar + ss.str() + ".pcd";
        else
            vstrLidarFile[i] = strPrefixLidar + ss.str() + ".bin";
    }
}