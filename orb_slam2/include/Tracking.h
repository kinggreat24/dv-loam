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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/imgproc/types_c.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>


#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "loam/MultiScanRegistration.h"
#include "Initializer.h"
#include "System.h"

#include "odomEstimationClass.h"


#include "lidar_sparse_align/SparseLidarAlign.h"
#include "lidar_sparse_align/SlideWindowSparseAlign.h"
#include "depth_clustering/projections/spherical_projection.h"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#ifdef USE_VGICP_CUDA
    #include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif


#include "LidarIris.h"

#include "ground_segmentation/ground_segmentation.h"

#include <vikit/pinhole_camera.h>

#include <mutex>

namespace ORB_SLAM2
{

class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

struct ORBParameters{
    // general parameters for the ORB detector
    int maxFrames, nFeatures, nLevels, iniThFAST, minThFAST;
    bool RGB;
    float scaleFactor, depthMapFactor, thDepth;
    // camera parameters
    int width, height;
    float fx, fy, cx, cy, baseline;
    float k1, k2, p1, p2, k3;

    Eigen::Matrix4f cameraLidarExtrinsic;

    //视觉激光直接法参数
    bool use_lidar = false;
    tracker_t tracker_;

    //激光点云lidar iris参数
    int iris_nscale;
    int iris_minWaveLength;
    float iris_mult;
    float iris_sigmaOnf;
    int iris_matchNum;

    //lidar feature
    std::string lidar_name;

    linefit_ground_segmentation::GroundSegmentationParams groundSegmentationParams;
};


class Tracking
{

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const int sensor, ORBParameters& parameters);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
    cv::Mat GrabImageMonocularLidar(const cv::Mat &im, const std::string & imLabel, const std::string& lidar_file, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetMinimumKeyFrames (int min_num_kf) {mnMinimumKeyFrames = min_num_kf;}

    cv::Mat ComputeF12(const cv::Mat& Tcw1, const cv::Mat& Tcw2, const cv::Mat& K1,const cv::Mat& K2);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);
    
    cv::Mat ComputeH12(const Frame&F1, const Frame &F2);
    void TrackFeaturesOpticalFlowPyrLK(const Frame &imgrayLast, const cv::Mat &imgrayCur, 
        std::vector<int>& feature_matches, const cv::Mat& Tcl = cv::Mat::eye(4,4,CV_32F));

    void ProcessMovingObject(const cv::Mat &imgrayLast, const cv::Mat &imgrayCur, const cv::Mat &dynamic_mask, cv::Mat& debug_image,bool flow_back=false);
    void ObjectAssociation(const Frame &LastFrame, const Frame &CurrentFrame,cv::Mat& out_img, bool use_epipolar_constraint=false);
    int SearchByTrackingHarris(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
    int SearchByTracking(const Frame &LastFrame, const Frame &CurrentFrame, const float th, const bool bMono);

    void DrawFeatureMatching(const Frame &frame1, const Frame &frame2, 
        const std::vector<cv::KeyPoint>& pts1, const std::vector<cv::KeyPoint>& pts2, 
        std::vector<cv::DMatch> &matches, cv::Mat &out_img);
    void ShowFeatureMatching(const Frame& frame1, const Frame& frame2, std::vector<cv::DMatch>& matches, cv::Mat& out_img);

    int GetCurrentKeyFrameId(){return mnLastKeyFrameId;}
public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

    SWOdomEstimationClass* mp_sw_optimizater;

    //Module Time
    std::vector<int64_t> mvTrackingTime;
    std::vector<int64_t> mvSWTrackingTime;
    std::vector<int64_t> mvScan2MapTime;
    std::vector<int64_t> mvScan2MapIndex;
protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    // Map initialization for monocular-LiDAR
    void VisualLidarInitialization();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    bool Scan2MapOptimization();
    void SearchLocalPoints();

    bool NeedCreateKeyFrame();
    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Numer of Keyframes a map has to have to not get a reset in the event of lost tracking.
    int mnMinimumKeyFrames;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    // Lidar feature extraction
    loam::RegistrationParams mRegistrationParams;
    loam::MultiScanRegistration* mpMultiScanRegistration;
    pcl::VoxelGrid<pcl::PointXYZI> m_down_sample_filter_surf;
    pcl::VoxelGrid<pcl::PointXYZI> m_down_sample_filter_corner;
    
    // linefit ground extraction
    linefit_ground_segmentation::GroundSegmentationParams mGroundSegmentationParams;
    linefit_ground_segmentation::GroundSegmentation* mpGroundSegmenter;

    // 激光点云全局描述子
    LidarIris* mpLidarIris;

    // Lidar range image 
    depth_clustering::SphericalProjection *mpSphericalProjection;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    // Initalization flag (only for monocular lidar)
    bool mbInitializationFlag;

    // Direct Visual Lidar Tracking
    SparseLidarAlign* mpSparseLidarAlign;

    std::vector<Frame> mvLocalKeyFrames;
    SlideWindowSparseAlign* mpSlideWindowOptimizater;


    // pcl::ApproximateVoxelGrid<pcl::PointXYZI> m_gicp_voxelgrid;
    fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI> m_fast_vgicp;
    pcl::VoxelGrid<pcl::PointXYZI> m_gicp_voxelgrid;
    // fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI> m_fast_vgicp;
    
    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<Frame*>    mvpLocalFrames;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // System
    System* mpSystem;

    //Drawers
    FrameDrawer* mpFrameDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    vk::PinholeCamera* mpPinholeCamera;

    //camera lidar extrinsic 
    Eigen::Matrix4f mCameraLidarExtrinsic;  //激光雷达到相机的外参变换

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;
    Sophus::SE3 mT_cur_last;      // 上一帧到当前帧的变换
    Sophus::SE3 mT_cur_ref;       // 关键帧到当前帧的变换

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    

    // These parameters are for the ORB features extractor
    int nFeatures;
    float fScaleFactor;
    int nLevels;
    int fIniThFAST;
    int fMinThFAST;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
