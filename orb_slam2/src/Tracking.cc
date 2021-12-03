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

#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include "CeresOptimizer.h"
#include "depth_clustering/projections/spherical_projection.h"

#include <iostream>

#include <mutex>

using namespace std;

#define FRAME_2_FRAME_TRACKING                       1
#define USE_GICP_OPTIMIZATION                        0
#define USE_LIDAR_SLIDE_WINDOW_OPTIMIZATION          0
#define USE_VISUAL_SLIDE_WINDOW_OPTIMIZATION         1
#define LOCAL_WINDOW_SIZE                            3
#define USE_SCAN2MAP_OPTIMIZATION                    1

namespace ORB_SLAM2
{

    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
                       Map *pMap, KeyFrameDatabase *pKFDB, const int sensor, ORBParameters &parameters)
        : mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
          mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mnLastKeyFrameId(0),
          mpFrameDrawer(pFrameDrawer), mpMap(pMap), mnLastRelocFrameId(0), mnMinimumKeyFrames(5), mbInitializationFlag(false)
    {
        //Unpack all the parameters from the parameters struct (this replaces loading in a second configuration file)
        mMaxFrames = parameters.maxFrames;
        mbRGB = parameters.RGB;
        mThDepth = parameters.thDepth;
        nFeatures = parameters.nFeatures;
        fScaleFactor = parameters.scaleFactor;
        nLevels = parameters.nLevels;
        fIniThFAST = parameters.iniThFAST;
        fMinThFAST = parameters.minThFAST;
        mDepthMapFactor = parameters.depthMapFactor;

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = parameters.fx;
        K.at<float>(1, 1) = parameters.fy;
        K.at<float>(0, 2) = parameters.cx;
        K.at<float>(1, 2) = parameters.cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = parameters.k1;
        DistCoef.at<float>(1) = parameters.k2;
        DistCoef.at<float>(2) = parameters.p1;
        DistCoef.at<float>(3) = parameters.p2;
        if (parameters.k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = parameters.k3;
        }
        DistCoef.copyTo(mDistCoef);

        //相机模型
        mpPinholeCamera = new vk::PinholeCamera(parameters.width, parameters.height, parameters.fx, parameters.fy,
                                                parameters.cx, parameters.cy, parameters.k1, parameters.k2, parameters.p1, parameters.p2, parameters.k3);

        // 视觉激光雷达直接法跟踪
        if (parameters.use_lidar)
        {
            cout << " - Lidar name: " << parameters.lidar_name << endl;

            // 相机与激光雷达的外参
            mCameraLidarExtrinsic = parameters.cameraLidarExtrinsic;

            cout << endl
                 << "Direct Sparse Visual odometry Parameters: " << endl;
            cout << "- max track level: " << parameters.tracker_.max_level << endl;
            cout << "- min track level: " << parameters.tracker_.min_level << endl;
            cout << "- max iteration: " << parameters.tracker_.max_iteration << endl;
            cout << "- weight function: " << parameters.tracker_.weight_function << std::endl;
            cout << "- scale function: " << parameters.tracker_.scale_estimator << std::endl;
            mpSparseLidarAlign       = new SparseLidarAlign(mpPinholeCamera, parameters.tracker_);
            mpSparseLidarAlign->setVerbose(false);
            mpSlideWindowOptimizater = new SlideWindowSparseAlign(mpPinholeCamera, parameters.tracker_);
            mpSlideWindowOptimizater->setVerbose(false);

            //地面提取参数
            mGroundSegmentationParams = parameters.groundSegmentationParams;
            cout << endl
                 << "linefit ground segmentation parameters: " << endl;
            cout << " - r_min_square: " << mGroundSegmentationParams.r_min_square << std::endl;
            cout << " - r_max_square: " << mGroundSegmentationParams.r_max_square << std::endl;
            cout << " - n_bins: " << mGroundSegmentationParams.n_bins << std::endl;
            cout << " - n_segments: " << mGroundSegmentationParams.n_segments << std::endl;
            cout << " - max_dist_to_line: " << mGroundSegmentationParams.max_dist_to_line << std::endl;
            cout << " - max_slope: " << mGroundSegmentationParams.max_slope << std::endl;
            cout << " - max_error_square: " << mGroundSegmentationParams.max_error_square << std::endl;
            cout << " - long_threshold: " << mGroundSegmentationParams.long_threshold << std::endl; // Distance at which points are considered far from each other.
            cout << " - max_long_height: " << mGroundSegmentationParams.max_long_height << std::endl;
            cout << " - sensor_height: " << mGroundSegmentationParams.sensor_height << std::endl;
            cout << " - line_search_angle: " << mGroundSegmentationParams.line_search_angle << std::endl;
            cout << " - n_threads: " << mGroundSegmentationParams.n_threads << std::endl;
            mpGroundSegmenter = new linefit_ground_segmentation::GroundSegmentation(mGroundSegmentationParams);

            //激光雷达投影
            depth_clustering::ProjectionParams::Ptr pLidarProjectionParams;
            if (parameters.lidar_name == "HDL-64E")
            {
                pLidarProjectionParams = depth_clustering::ProjectionParams::HDL_64_EQUAL();
                // pLidarProjectionParams = depth_clustering::ProjectionParams::HDL_64_EQUAL(4000,64);
            }
            else if (parameters.lidar_name == "HDL-32")
                pLidarProjectionParams = depth_clustering::ProjectionParams::HDL_32();
            else if (parameters.lidar_name == "VLP-16")
                pLidarProjectionParams = depth_clustering::ProjectionParams::VLP_16();
            mpSphericalProjection = new depth_clustering::SphericalProjection(*pLidarProjectionParams);

            //lidar feature extractor
            mpMultiScanRegistration = new loam::MultiScanRegistration();
            mpMultiScanRegistration->setupRegistrationParams(parameters.lidar_name, mRegistrationParams);
            cout << endl
                 << "Lidar parameters: " << endl;
            cout << " - scanPeriod: " << mRegistrationParams.scanPeriod << endl;
            cout << " - nFeatureRegions: " << mRegistrationParams.nFeatureRegions << endl;
            cout << " - curvatureRegion: " << mRegistrationParams.curvatureRegion << endl;
            cout << " - maxCornerSharp: " << mRegistrationParams.maxCornerSharp << endl;
            cout << " - maxSurfaceFlat: " << mRegistrationParams.maxSurfaceFlat << endl;
            cout << " - lessFlatFilterSize: " << mRegistrationParams.lessFlatFilterSize << endl;
            cout << " - surfaceCurvatureThreshold: " << mRegistrationParams.surfaceCurvatureThreshold << endl;

            m_down_sample_filter_surf.setLeafSize(0.8, 0.8, 0.8);
            m_down_sample_filter_corner.setLeafSize(0.4, 0.4, 0.4);

            // 激光雷达局部icp优化
#if (USE_GICP_OPTIMIZATION)
            double downsample_resolution = 0.5;
            m_gicp_voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            m_fast_vgicp.setMaxCorrespondenceDistance(0.1);
            m_fast_vgicp.setNumThreads(omp_get_max_threads());
            std::cout << "***********         omp_get_max_threads: " << omp_get_max_threads() << "          *************" << std::endl;
#endif//

            mp_sw_optimizater = new SWOdomEstimationClass();
            mp_sw_optimizater->init(0.4);

            // 激光点云闭环检测
            //Lidar loopclosure detector
            cout << endl
                 << "Lidar Iris: " << endl;
            cout << " - nscale: " << parameters.iris_nscale << endl;
            cout << " - minWaveLength: " << parameters.iris_minWaveLength << endl;
            cout << " - mult: " << parameters.iris_mult << endl;
            cout << " - sigmaOnf: " << parameters.iris_sigmaOnf << endl;
            cout << " - matchNum: " << parameters.iris_matchNum << endl;
            mpLidarIris = new LidarIris(parameters.iris_nscale, parameters.iris_minWaveLength, parameters.iris_mult,
                                        parameters.iris_sigmaOnf, parameters.iris_matchNum);
        }

        mbf = parameters.baseline;

        // Max/Min Frames to insert keyframes and to check relocalization
        mMinFrames = 0;

        cout << endl
             << "Camera Parameters: " << endl;
        cout << "- fx: " << parameters.fx << endl;
        cout << "- fy: " << parameters.fy << endl;
        cout << "- cx: " << parameters.cx << endl;
        cout << "- cy: " << parameters.cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if (DistCoef.rows == 5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- fps: " << mMaxFrames << endl;
        cout << "- bf: " << mbf << endl;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::MONOCULAR || sensor == System::VISUAL_LIDAR)
            mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl
             << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        if (sensor == System::STEREO || sensor == System::RGBD)
        {
            mThDepth = mbf * (float)mThDepth / parameters.fx;
            cout << endl
                 << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }

        if (sensor == System::RGBD)
        {
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }
    }

    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing = pLoopClosing;
    }

    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
    {
        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
    {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
    {
        mImGray = im;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

    cv::Mat Tracking::GrabImageMonocularLidar(const cv::Mat &im, const std::string &imLabel, const std::string &lidar_file, const double &timestamp)
    {
        mImGray = im;

        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, imLabel, lidar_file, timestamp, mpORBextractorLeft, mpORBVocabulary, mpSphericalProjection, mpMultiScanRegistration, mGroundSegmentationParams, mpPinholeCamera, mK, mDistCoef, mCameraLidarExtrinsic, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, imLabel, lidar_file, timestamp, mpORBextractorLeft, mpORBVocabulary, mpSphericalProjection, mpMultiScanRegistration, mGroundSegmentationParams, mpPinholeCamera, mK, mDistCoef, mCameraLidarExtrinsic, mbf, mThDepth);

    #if(USE_OPTICAL_FLOW)
        if (mCurrentFrame.mnId != 0)
        {
            // cv::Mat obj_associa;
            // ObjectAssociation(mLastFrame, mCurrentFrame, obj_associa, true);
            // sprintf(file_name, "/home/kinggreat24/pc/dynamic_obj/association/%d_associa.png", mCurrentFrame.mnId);
            // cv::imwrite(file_name, 255*obj_associa);

            cv::Mat last_image = mLastFrame.mvImgPyramid[0].clone();
            cv::Mat cur_image  = mCurrentFrame.mvImgPyramid[0].clone();
            last_image.convertTo(last_image, CV_8U, 255.0);
            cur_image.convertTo(cur_image, CV_8U, 255.0);
            cv::Mat object_mask = mLastFrame.GenerateObjectMask();
            cv::Mat debug_image;
            ProcessMovingObject(last_image,cur_image,object_mask,debug_image,true);
            char file_name[128] = {0};
            sprintf(file_name, "/home/kinggreat24/pc/dynamic_obj/mask/%d_debug_image.png", mCurrentFrame.mnId);
            cv::imwrite(file_name, debug_image);
        }
    #endif//USE_OPTICAL_FLOW


        Track();

        return mCurrentFrame.mTcw.clone();
    }

    void Tracking::Track()
    {
        if (mState == NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        if (mState == NOT_INITIALIZED)
        {
            if (mSensor == System::STEREO || mSensor == System::RGBD)
                StereoInitialization();
            else if (mSensor == System::MONOCULAR)
            {
                MonocularInitialization();
            }
            else if (mSensor == System::VISUAL_LIDAR)
            {
                VisualLidarInitialization();

                mT_cur_ref = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
            }

            std::cout<<"frame drawer begin"<<std::endl;
            mpFrameDrawer->Update(this);
            std::cout<<"frame drawer over"<<std::endl;
            if (mState != OK)
                return;
        }
        else
        {
            // System is initialized. Track Frame.
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if (!mbOnlyTracking)
            {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.
                if (mState == OK)
                {
                    // (1) Direct visual lidar odometry
#if (FRAME_2_FRAME_TRACKING)
                    // std::cout << "Direct tracking with last frame" << std::endl;

                    cv::Mat Tcl_init = cv::Mat::eye(4, 4, CV_32FC1);
                    if (mVelocity.empty())
                    {
                        mT_cur_last = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                        // TrackReferenceKeyFrame();
                    }
                    else
                    {
                        Tcl_init = mVelocity;
                        mT_cur_last = Converter::toSophusSE3(mVelocity);
                        // TrackWithMotionModel();
                    }

                    // 直接法跟踪(frame-2-frame)
                    auto tic_lidar = loam::timestamp_now();
                    bOK = mpSparseLidarAlign->tracking(&mLastFrame, &mCurrentFrame, mT_cur_last);
                    auto toc_lidar = loam::timestamp_now();
                    int64_t time = (toc_lidar - tic_lidar) * 1e-3;
                    mvTrackingTime.push_back(time);
                    // std::cout << "direct tracking time: " << time << " ms" << std::endl;
                    // std::cout << "Twc_cur vlo: " << std::endl
                    //           << mT_cur_last.matrix() << std::endl;
                    Eigen::Matrix4d current_tcw = mT_cur_last.matrix() * Converter::toMatrix4d(mLastFrame.mTcw);
                    mCurrentFrame.SetPose(Converter::toCvMat(current_tcw));

#else //FRAME_2_KEYFRAME_TRACKING

                    std::cout << "Direct tracking with last keyframe" << std::endl;
                    // 根据运动模型估计当前帧的位置信息，　并计算关键帧到当前帧的变换
                    if (mVelocity.empty())
                        mT_cur_ref = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                    else
                        mT_cur_ref = Converter::toSophusSE3(mVelocity * mLastFrame.mTcw * mpLastKeyFrame->GetPoseInverse());

                    // 直接法跟踪(frame-2-keyframe)
                    auto tic_lidar = loam::timestamp_now();
                    bOK = mpSparseLidarAlign->tracking(mpLastKeyFrame, &mCurrentFrame, mT_cur_ref);
                    auto toc_lidar = loam::timestamp_now();
                    int64_t time = (toc_lidar - tic_lidar) * 1e-3;
                    mvTrackingTime.push_back(time);
                    std::cout << "direct tracking with keyframe time: " << time << " ms" << std::endl;

                    Eigen::Matrix4d current_tcw = mT_cur_ref.matrix() * Converter::toMatrix4d(mpLastKeyFrame->GetPose());
                    mCurrentFrame.SetPose(Converter::toCvMat(current_tcw));

#endif //FRAME_2_FRAME_TRACKING
                }
                else
                {
                    bOK = Relocalization();
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if (!mbOnlyTracking)
            {
                // std::cout << "Track local map, slide window optimization" << std::endl;
                if (bOK)
                {

#if(USE_GICP_OPTIMIZATION)
                    // Voxel-GICP Optimization
                    //设置source点云
                    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filter_source_pc(new pcl::PointCloud<pcl::PointXYZI>());
                    m_gicp_voxelgrid.setInputCloud(mCurrentFrame.mpLidarPointCloudCamera);
                    m_gicp_voxelgrid.filter(*voxel_filter_source_pc);
                    m_fast_vgicp.setInputSource(voxel_filter_source_pc);

                    //设置目标点云
                    pcl::PointCloud<pcl::PointXYZI>::Ptr local_target_pc(new pcl::PointCloud<pcl::PointXYZI>());
                    for(auto iter = mvpLocalKeyFrames.begin(); iter != mvpLocalKeyFrames.end(); iter++)
                    {
                        Eigen::Matrix4d Twc = Converter::toMatrix4d((*iter)->GetPoseInverse());
                        pcl::PointCloud<pcl::PointXYZI>::Ptr local_world_pc_surf(new pcl::PointCloud<pcl::PointXYZI>());
                        pcl::transformPointCloud(*((*iter)->mpSurfPointsLessFlat), *local_world_pc_surf, Twc);
                        *local_target_pc += *local_world_pc_surf;

                        pcl::PointCloud<pcl::PointXYZI>::Ptr local_world_pc_ground(new pcl::PointCloud<pcl::PointXYZI>());
                        pcl::transformPointCloud(*((*iter)->mpGroundPoints), *local_world_pc_ground, Twc);
                        *local_target_pc += *local_world_pc_ground;
                    }
                    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filter_target_pc(new pcl::PointCloud<pcl::PointXYZI>());
                    m_gicp_voxelgrid.setInputCloud(local_target_pc);
                    m_gicp_voxelgrid.filter(*voxel_filter_target_pc);
                    m_fast_vgicp.setInputTarget(voxel_filter_target_pc);

                    //vgicp 对齐
                    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pc(new pcl::PointCloud<pcl::PointXYZI>);
                    auto tic_vgicp = loam::timestamp_now();
                    m_fast_vgicp.align(*aligned_pc, twc_cur);
                    auto toc_vgicp = loam::timestamp_now();
                    int64_t time_vgicp = (toc_vgicp-tic_vgicp)*1e-3;
                    mvSWTrackingTime.push_back(time_vgicp);
                    std::cout<<"slide window optimization time: "<<time_vgicp<<std::endl;

                    //设置当前帧的姿态
                    Eigen::Matrix4f vgicp_results_Tcw = m_fast_vgicp.getFinalTransformation().inverse();
                    mCurrentFrame.SetPose(Converter::toCvMat(vgicp_results_Tcw));

                    char static_pc_name[128]={0};
                    sprintf(static_pc_name,"/home/kinggreat24/pc/%d_gicp_source.pcd",mCurrentFrame.mnId);
                    pcl::io::savePCDFileASCII(static_pc_name, *voxel_filter_source_pc);//将点云保存到PCD文件中

                    sprintf(static_pc_name,"/home/kinggreat24/pc/%d_gicp_target.pcd",mCurrentFrame.mnId);
                    pcl::io::savePCDFileASCII(static_pc_name, *voxel_filter_target_pc);//将点云保存到PCD文件中

                    sprintf(static_pc_name,"/home/kinggreat24/pc/%d_gicp_align.pcd",mCurrentFrame.mnId);
                    pcl::io::savePCDFileASCII(static_pc_name, *aligned_pc);//将点云保存到PCD文件中

                    std::cout << "mTcw pose after local mapping optimization: " << std::endl
                              << mCurrentFrame.mTcw << std::endl;
#endif//USE_GICP_OPTIMIZATION          


#if(USE_LIDAR_SLIDE_WINDOW_OPTIMIZATION)
                    Eigen::Matrix4d twc_cur = Converter::toMatrix4d(mCurrentFrame.mTcw).inverse();

                    auto tic_vgicp = loam::timestamp_now();
                    mp_sw_optimizater->updatePointsToMap(twc_cur,
                        mCurrentFrame.mpCornerPointsLessSharp, 
                        mCurrentFrame.mpSurfPointsLessFlat);
                    
                    auto toc_vgicp = loam::timestamp_now();
                    int64_t time_vgicp = (toc_vgicp - tic_vgicp) * 1e-3;
                    mvSWTrackingTime.push_back(time_vgicp);

                    Eigen::Matrix4d tcw_cur_opt = twc_cur.inverse();
                    mCurrentFrame.SetPose(Converter::toCvMat(tcw_cur_opt));    
#endif//USE_LIDAR_SLIDE_WINDOW_OPTIMIZATION

#if(USE_VISUAL_SLIDE_WINDOW_OPTIMIZATION)

                    if(mvLocalKeyFrames.size() > 0)
                    {
                        auto tic_vgicp = loam::timestamp_now();

                        Sophus::SE3 Tcw_cur = Converter::toSophusSE3(mCurrentFrame.mTcw);
                        
                        //与当前关键帧进行优化
                        // set_color(STDOUT_FILENO, ORB_SLAM2::LBLUE);
                        // Frame refFrame = mvLocalKeyFrames.back();
                        // Sophus::SE3 Tcw_ref = Converter::toSophusSE3(refFrame.mTcw);
                        // Sophus::SE3 T_cur_ref = Tcw_cur * Tcw_ref.inverse();
                        // mpSparseLidarAlign->tracking(&refFrame, &mCurrentFrame, T_cur_ref);
                        // Sophus::SE3 Tcw_cur_opti_ = T_cur_ref * Tcw_ref;
                        // std::cout<<"Opti frame-to-keyframe Tcw_cur_opti_: "<<std::endl<<Tcw_cur_opti_.matrix()<<std::endl;

                        //滑动窗口优化
                        set_color(STDOUT_FILENO, ORB_SLAM2::YELLOW);
                        Sophus::SE3 Tcw_cur_opti =  Tcw_cur;
                        // std::cout<<"Init Tcw_cur: "<<std::endl<<Tcw_cur.matrix()<<std::endl;
                        mpSlideWindowOptimizater->slidewindow_optimization(mvLocalKeyFrames,
                            &mCurrentFrame,Tcw_cur_opti);
                        // std::cout<<"Opti Tcw_cur_opti: "<<std::endl<<Tcw_cur_opti.matrix()<<std::endl;

                        //判断优化的结果是否异常
                        Sophus::SE3 diff_manifold = Tcw_cur * Tcw_cur_opti.inverse();
                        if(diff_manifold.translation().norm() > 0.5)
                        {
                            std::cout<<"*********   optimization changed, use feature based optimization     **********"<<std::endl;
                            Tcw_cur_opti = Tcw_cur;
                        }

                        reset_color(STDOUT_FILENO);

                        auto toc_vgicp = loam::timestamp_now();
                        int64_t time_vgicp = (toc_vgicp - tic_vgicp) * 1e-3;
                        mvSWTrackingTime.push_back(time_vgicp);

                        mCurrentFrame.SetPose(Converter::toCvMat(Tcw_cur_opti.matrix()));
                    }
#endif // end of USE_VISUAL_SLIDE_WINDOW_OPTIMIZATION

                }
            }

            if (bOK)
                mState = OK;
            else
                mState = LOST;

            if (NeedCreateKeyFrame() || mCurrentFrame.mnId <= 5)
            {
                std::cout << "**********           create new keyframe          ************" << std::endl;
            
            #if(USE_SCAN2MAP_OPTIMIZATION)
                //对当前帧进行激光scan2map优化
                auto tic_scan2map = loam::timestamp_now();
                
                Eigen::Matrix4d Twc_cur = Converter::toMatrix4d(mCurrentFrame.mTcw).inverse();
                mp_sw_optimizater->updatePointsToMap(Twc_cur,mCurrentFrame.mpCornerPointsLessSharp, 
                        mCurrentFrame.mpSurfPointsLessFlat);
                Eigen::Matrix4d Tcw_lidar_opti_cur = Twc_cur.inverse();
                mCurrentFrame.SetPose(Converter::toCvMat(Tcw_lidar_opti_cur));

                auto toc_scan2map = loam::timestamp_now();
                int64_t time_scan2map = (toc_scan2map - tic_scan2map) * 1e-3;
                mvScan2MapTime.push_back(time_scan2map);
                mvScan2MapIndex.push_back(mCurrentFrame.mnId);
            #endif// end of USE_SCAN2MAP_OPTIMIZATION

                //创建关键帧
                KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
                if (pKF->mBowVec.empty())
                    pKF->ComputeBoW();

                mpReferenceKF = pKF;
                mCurrentFrame.mpReferenceKF = pKF;

                //计算IRIS特征
                cv::Mat1b li1 = mpLidarIris->GetIris(*(mCurrentFrame.mpLidarPointCloudRaw));
                pKF->mLidarIrisFeature = mpLidarIris->GetFeature(li1);

                // 添加到回环检测中
                // mpLoopClosing->InsertKeyFrame(pKF);

                // 局部关键帧
                if (mvpLocalKeyFrames.size() >= 3)
                {
                    mvpLocalKeyFrames.erase(mvpLocalKeyFrames.begin());
                }
                mvpLocalKeyFrames.push_back(pKF);

                mnLastKeyFrameId = mCurrentFrame.mnId;
                mpLastKeyFrame = pKF;

                mpMap->AddKeyFrame(pKF);


                //局部关键帧窗口
                if(mvLocalKeyFrames.size() >= LOCAL_WINDOW_SIZE)
                    mvLocalKeyFrames.erase(mvLocalKeyFrames.begin());
                mvLocalKeyFrames.push_back(mCurrentFrame);
            }

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if (bOK)
            {
                // Update motion model
                if (!mLastFrame.mTcw.empty())
                {
                    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                    mVelocity = mCurrentFrame.mTcw * LastTwc;
                }
                else
                    mVelocity = cv::Mat();
            }

            // Reset if the camera get lost soon after initialization
            if (mState == LOST)
            {
                if (mpMap->KeyFramesInMap() <= mnMinimumKeyFrames)
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if (!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }
    }

    cv::Mat Tracking::ComputeF12(const cv::Mat &Tcw1, const cv::Mat &Tcw2, const cv::Mat &K1, const cv::Mat &K2)
    {
        cv::Mat R1w = Tcw1.rowRange(0, 3).colRange(0, 3);
        cv::Mat t1w = Tcw1.rowRange(0, 3).col(3);
        cv::Mat R2w = Tcw2.rowRange(0, 3).colRange(0, 3);
        cv::Mat t2w = Tcw2.rowRange(0, 3).col(3);

        cv::Mat R12 = R1w * R2w.t();
        cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;

        cv::Mat t12x = SkewSymmetricMatrix(t12);

        // const cv::Mat &K1 = pKF1->mK;
        // const cv::Mat &K2 = pKF2->mK;

        return K1.t().inv() * t12x * R12 * K2.inv();
    }

    void Tracking::StereoInitialization()
    {
        if (mCurrentFrame.N > 500)
        {
            // Set Frame pose to the origin
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            // Create KeyFrame
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }

            cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpMap->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            //mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState = OK;
        }
    }

    void Tracking::MonocularInitialization()
    {

        if (!mpInitializer)
        {
            // Set Reference Frame
            if (mCurrentFrame.mvKeys.size() > 100)
            {
                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

                if (mpInitializer)
                    delete mpInitializer;

                mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                return;
            }
        }
        else
        {
            // Try to initialize
            if ((int)mCurrentFrame.mvKeys.size() <= 100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
                return;
            }

            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);
            std::cout << "nmatches: " << nmatches << std::endl;

            // Check if there are enough correspondences
            if (nmatches < 100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                return;
            }

            cv::Mat Rcw;                 // Current Camera Rotation
            cv::Mat tcw;                 // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
            {
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
                {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                mCurrentFrame.SetPose(Tcw);

                CreateInitialMapMonocular();
            }
        }
    }

    void Tracking::CreateInitialMapMonocular()
    {
        // Create KeyFrames
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Create MapPoints and asscoiate to keyframes
        for (size_t i = 0; i < mvIniMatches.size(); i++)
        {
            if (mvIniMatches[i] < 0)
                continue;

            //Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);

            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpMap->AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

        Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

        // Set median depth to 1
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
        {
            cout << "Wrong initialization, reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
        {
            if (vpAllMapPoints[iMP])
            {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        //mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }

    // 视觉激光雷达初始化
    void Tracking::VisualLidarInitialization()
    {
        set_color(STDOUT_FILENO, ORB_SLAM2::GREEN);

        if(!mbInitializationFlag)
        {
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            //深度拟合
            std::cout<<"Feature Depth init"<<std::endl;
            auto tic_depth = loam::timestamp_now();
            
            GEOM_FADE25D::Fade_2D * pdt_ref = mCurrentFrame.CreateTerrain(mCurrentFrame.mpLidarPointCloudCamera);
            for(size_t i=0;i<mCurrentFrame.mvKeysUn.size();i++)
            {
                cv::Point2f kpt = mCurrentFrame.mvKeysUn.at(i).pt;
                mCurrentFrame.mvDepth[i] = mCurrentFrame.DepthFitting(pdt_ref,kpt);
            }
            
            auto toc_depth = loam::timestamp_now();
            int64_t time = (toc_depth - tic_depth) * 1e-3;
            std::cout<<"depth fitting time: "<<time<<" ms"<<std::endl;

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame    = Frame(mCurrentFrame);

            mbInitializationFlag = true;
            return;
        }
        else
        {
            // Monocular Lidar Initialization
            // Find correspondences
            ORBmatcher matcher(0.9, true);
            std::vector<cv::DMatch> vInitMatches;
            int nmatches = matcher.SearchByBoW(mInitialFrame, mCurrentFrame, vInitMatches);
            std::cout << "feature nmatches: " << nmatches << std::endl;

            // Ransac to filter outlier
            std::vector<cv::Point2f> vPointsRef;
            std::vector<cv::Point2f> vPointsCurrent;
            std::vector<cv::DMatch> vInitMatchesValid;
            for(size_t i=0;i<vInitMatches.size();i++)
            {
                cv::DMatch dm = vInitMatches.at(i);

                if(dm.trainIdx < 0)
                    continue;

                vPointsRef.push_back( mInitialFrame.mvKeysUn[dm.queryIdx].pt);
                vPointsCurrent.push_back( mCurrentFrame.mvKeysUn[dm.trainIdx].pt);

                vInitMatchesValid.push_back(dm);
            }
            

            std::vector<cv::DMatch> vFilterInitMatches;
            std::vector<uchar>      m_RANSACStatus;
            cv::Mat F12 = cv::findFundamentalMat (vPointsCurrent, vPointsRef, m_RANSACStatus, CV_FM_RANSAC, 3.0 );
            
            std::vector<Eigen::Vector3d> x_3ds_ref;
            std::vector<cv::KeyPoint> x_2ds_cur;
            for(size_t i=0;i<m_RANSACStatus.size();i++)
            {
                if(m_RANSACStatus[i] == 0)
                {
                    nmatches--;
                }
                else
                {
                    cv::DMatch dm = vInitMatchesValid[i];
                    vFilterInitMatches.push_back(dm);
                    
                    float depth = mInitialFrame.mvDepth[dm.queryIdx];
                    if(depth > 0)
                    {
                        //参考帧3D空间点
                        Eigen::Vector2d k_pt(vPointsRef[i].x,vPointsRef[i].y);
                        Eigen::Vector3d p_3d = mpPinholeCamera->cam2world(k_pt) * depth;
                        x_3ds_ref.push_back(p_3d);

                        //当前帧2D点
                        x_2ds_cur.push_back(mCurrentFrame.mvKeysUn[dm.trainIdx]);
                    }
                }
            }
            // 特征点法计算相对运动
            std::cout<<"Inlier points3d size after RansacF: "<<x_3ds_ref.size()<<std::endl;
            cv::Mat T_cur_ref_mat = cv::Mat::eye(4,4,CV_32F);
            int nInlier = Optimizer::PoseOptimization(x_3ds_ref,x_2ds_cur,mCurrentFrame.mvInvLevelSigma2,mK,T_cur_ref_mat);
            std::cout<<"Inlier size: "<<nInlier
                <<" optimized Twc: "<<std::endl<<T_cur_ref_mat.inv()<<std::endl;
 

            //视觉激光直接法计算运动
            // Sophus::SE3 T_cur_ref = Sophus::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
            Sophus::SE3 T_cur_ref = Converter::toSophusSE3(T_cur_ref_mat);
            mpSparseLidarAlign->tracking(&mInitialFrame,&mCurrentFrame,T_cur_ref);
            std::cout<<"direct tracking results: "<<std::endl<<T_cur_ref.matrix()<<std::endl;


            //激光计算相对运动
            Eigen::Matrix4d Twc_cur  = T_cur_ref.matrix().inverse();
            std::cout<<"Twc_cur init: "<<std::endl<<Twc_cur.matrix()<<std::endl;
            mp_sw_optimizater->initMapWithPoints(mInitialFrame.mpCornerPointsLessSharp,
                mInitialFrame.mpSurfPointsLessFlat);
            mp_sw_optimizater->updatePointsToMap(Twc_cur, mCurrentFrame.mpCornerPointsLessSharp,
                mCurrentFrame.mpSurfPointsLessFlat);
            std::cout<<"lidar optimization results: "<<std::endl<<Twc_cur<<std::endl;
            

            // 特征点匹配结果
            std::cout<<" "<<nmatches<<std::endl;
            cv::Mat feature_match_image;
            DrawFeatureMatching(mInitialFrame,mCurrentFrame,mInitialFrame.mvKeys,mCurrentFrame.mvKeys,vFilterInitMatches,feature_match_image);
            cv::imwrite("/home/kinggreat24/pc/init_feature_match.png",feature_match_image);
            // cv::imshow("feature_match",feature_match_image);
            // cv::waitKey(0);

            mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            Eigen::Matrix4d Tcw_cur = Twc_cur.inverse();
            mCurrentFrame.SetPose(Converter::toCvMat(Tcw_cur));

            //添加到系统中
            // Create KeyFrames
            KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
            KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

            // 计算词袋向量
            if (pKFini->mBowVec.size() <= 0)
                pKFini->ComputeBoW();
            if (pKFcur->mBowVec.size() <= 0)
                pKFcur->ComputeBoW();

            //计算IRIS特征
            cv::Mat1b li1 = mpLidarIris->GetIris(*(mInitialFrame.mpLidarPointCloudRaw));
            pKFini->mLidarIrisFeature = mpLidarIris->GetFeature(li1);

            cv::Mat1b li2 = mpLidarIris->GetIris(*(mCurrentFrame.mpLidarPointCloudRaw));
            pKFcur->mLidarIrisFeature = mpLidarIris->GetFeature(li2);

            // Insert KFs in the map
            mpMap->AddKeyFrame(pKFini);
            mpMap->AddKeyFrame(pKFcur);

            // 添加到闭环检测模块中
            // mpLoopClosing->InsertKeyFrame(pKFini);
            // mpLoopClosing->InsertKeyFrame(pKFcur);

            // slide keyframe window
            mvLocalKeyFrames.push_back(mInitialFrame);
            mvLocalKeyFrames.push_back(mCurrentFrame);


            mpReferenceKF               = pKFcur;
            mCurrentFrame.mpReferenceKF = pKFcur;
            mnLastKeyFrameId            = mCurrentFrame.mnId;
            mLastFrame                  = Frame(mCurrentFrame);
            mpLastKeyFrame              = pKFcur;
            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mState = OK;
        }
        std::cout<<"Tracking::VisualLidarInitialization over"<<std::endl;
        reset_color(STDOUT_FILENO);

//         mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

//         mvLocalKeyFrames.push_back(mCurrentFrame);

//         // Create KeyFrames
//         KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

//         //计算词袋向量
//         if (mCurrentFrame.mBowVec.size() <= 0)
//             pKFini->ComputeBoW();

//         //添加到地图中
//         mpMap->AddKeyFrame(pKFini);

//         //计算IRIS特征
//         cv::Mat1b li1 = mpLidarIris->GetIris(*(mCurrentFrame.mpLidarPointCloudRaw));
//         pKFini->mLidarIrisFeature = mpLidarIris->GetFeature(li1);

        

//         // 局部关键帧
//         mvpLocalKeyFrames.push_back(pKFini);

//         mpReferenceKF = pKFini;
//         mCurrentFrame.mpReferenceKF = pKFini;
//         mnLastKeyFrameId = mCurrentFrame.mnId;
//         mLastFrame = Frame(mCurrentFrame);
//         mpLastKeyFrame = pKFini;
//         mpMap->mvpKeyFrameOrigins.push_back(pKFini);

//         mState = OK;
    }

    void Tracking::CheckReplacedInLastFrame()
    {
        for (int i = 0; i < mLastFrame.N; i++)
        {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP)
            {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    bool Tracking::TrackReferenceKeyFrame()
    {
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (nmatches < 15)
            return false;

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);

        Optimizer::PoseOptimization(&mCurrentFrame);

        std::cout << "optimization result: " << std::endl
                  << mCurrentFrame.mTcw << std::endl;

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap >= 10;
    }

    void Tracking::UpdateLastFrame()
    {
        // Update pose according to reference keyframe
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr * pRef->GetPose());

        if (mnLastKeyFrameId == mLastFrame.mnId || (mSensor == System::MONOCULAR || mSensor == System::VISUAL_LIDAR) || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for (int i = 0; i < mLastFrame.N; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];
            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1)
            {
                bCreateNew = true;
            }

            if (bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                mLastFrame.mvpMapPoints[i] = pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;
        }
    }

    bool Tracking::TrackWithMotionModel()
    {
        ORBmatcher matcher(0.9, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();

        mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;
        if (mSensor != System::STEREO)
            th = 15;
        else
            th = 7;
        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR || mSensor == System::VISUAL_LIDAR);

        // If few matches, uses a wider window search
        if (nmatches < 20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR || mSensor == System::VISUAL_LIDAR);
        }
        std::cout << "Tracking::TrackWithMotionModel nmatches: " << nmatches << std::endl;
        if (nmatches < 20)
            return false;

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);
        std::cout << "optimization result: " << std::endl
                  << mCurrentFrame.mTcw << std::endl;

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        if (mbOnlyTracking)
        {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        return nmatchesMap >= 10;
    }

    //帧到地图配准
    bool Tracking::Scan2MapOptimization()
    {
        Eigen::Matrix4d Twc_cur = Converter::toMatrix4d(mCurrentFrame.mTcw.inv());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pCornerStack(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pSurfStack(new pcl::PointCloud<pcl::PointXYZI>());
        m_down_sample_filter_corner.setInputCloud(mCurrentFrame.mpCornerPointsLessSharp);
        m_down_sample_filter_corner.filter(*pCornerStack);
        m_down_sample_filter_surf.setInputCloud(mCurrentFrame.mpSurfPointsLessFlat);
        m_down_sample_filter_surf.filter(*pSurfStack);
        // ToDo:: 提取DGP特征

        std::cout << "pCornerStack size: " << pCornerStack->size() << std::endl;
        std::cout << "pSurfStack size: " << pSurfStack->size() << std::endl;

        //获取局部地图
        auto tic_map = loam::timestamp_now();
        pcl::PointCloud<pcl::PointXYZI>::Ptr pCornerMap(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pSurfMap(new pcl::PointCloud<pcl::PointXYZI>());
        mpMap->GetLocalCellMap(Twc_cur, 100, pCornerMap, pSurfMap);
        auto toc_map = loam::timestamp_now();
        int64_t map_time = (toc_map - tic_map) * 1e-3;
        std::cout << "***********************                get lidar map time: " << map_time << " ms" << std::endl;

        auto tic_lidar = loam::timestamp_now();
        static int max_iteration = 10;
        CeresOptimizer::PoseOptimization(Twc_cur,
           pCornerMap, pSurfMap, pCornerStack, pSurfStack, max_iteration--);
        if (max_iteration < 3)
            max_iteration = 3;

        std::cout << "scan2map optimization Twc_cur: " << std::endl
                  << Twc_cur.matrix() << std::endl;
        auto toc_lidar = loam::timestamp_now();
        int64_t time = (toc_lidar - tic_lidar) * 1e-3;
        std::cout << "time: " << time << " ms" << std::endl;

        //
        Eigen::Matrix4d Tcw = Twc_cur.inverse();
        cv::Mat Tcw_cur = Converter::toCvMat(Tcw);
        mCurrentFrame.SetPose(Tcw_cur);
    }

    bool Tracking::TrackLocalMap()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        UpdateLocalMap();

        SearchLocalPoints();

        // Optimize Pose
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking)
                    {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            return false;

        if (mnMatchesInliers < 30)
            return false;
        else
            return true;
    }

    bool Tracking::NeedCreateKeyFrame()
    {
        if (mbOnlyTracking)
            return false;

        // const int nKFs = mpMap->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        // if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        //     return false;

        // 计算与关键帧之间的差别

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion (fps)
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;

        // Condition 1b: 与当前关键帧的视角重叠率
        const float radio_threshold = 0.70;
        Eigen::Matrix4d T_cur_ref = Converter::toMatrix4d(mCurrentFrame.mTcw * mpLastKeyFrame->GetPoseInverse());
        Sophus::SE3 T_cur_ref_sophus(T_cur_ref.block<3, 3>(0, 0), T_cur_ref.block<3, 1>(0, 3));
        float fov_radio = mpLastKeyFrame->get_visible_ratio(T_cur_ref_sophus);
        const bool c1b = fov_radio < radio_threshold ? true : false;

        if (c1a || c1b)
            return true;
        else
            return false;
    }

    bool Tracking::NeedNewKeyFrame()
    {
        if (mbOnlyTracking)
            return false;

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
            return false;

        const int nKFs = mpMap->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
            return false;

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;
        if (mSensor != System::MONOCULAR && mSensor != System::VISUAL_LIDAR)
        {
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Thresholds
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        if (mSensor == System::MONOCULAR || mSensor == System::VISUAL_LIDAR)
            thRefRatio = 0.9f;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
        //Condition 1c: tracking is weak
        const bool c1c = (mSensor != System::MONOCULAR && mSensor != System::VISUAL_LIDAR) && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

        if ((c1a || c1b || c1c) && c2)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if (bLocalMappingIdle)
            {
                return true;
            }
            else
            {
                mpLocalMapper->InterruptBA();
                if (mSensor != System::MONOCULAR && mSensor != System::VISUAL_LIDAR)
                {
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true;
                    else
                        return false;
                }
                else
                    return false;
            }
        }
        else
            return false;
    }

    void Tracking::CreateNewKeyFrame()
    {
        if (!mpLocalMapper->SetNotStop(true))
            return;

        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if (mSensor != System::MONOCULAR && mSensor != System::VISUAL_LIDAR)
        {
            mCurrentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float, int>> vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty())
            {
                sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew)
                    {
                        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                        pNewMP->AddObservation(pKF, i);
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++;
                    }

                    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                        break;
                }
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);

        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }

    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
            {
                if (pMP->isBad())
                {
                    *vit = static_cast<MapPoint *>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (mCurrentFrame.isInFrustum(pMP, 0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if (nToMatch > 0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            if (mSensor == System::RGBD)
                th = 3;
            // If the camera has been relocalised recently, perform a coarser search
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;
            matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        mvpLocalMapPoints.clear();

        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
            {
                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }

    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame *, int> keyframeCounter;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                    for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }

        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
        {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max)
            {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
            {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad())
                {
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
            {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad())
                {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent)
            {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        if (pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::Relocalization()
    {
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

        if (vpCandidateKFs.empty())
            return false;

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nKFs; i++)
            {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if (nGood < 10)
                        continue;

                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if (nGood < 50)
                    {
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                        if (nadditional + nGood >= 50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50)
                            {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                                // Final optimization
                                if (nGood + nadditional >= 50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }

                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if (!bMatch)
        {
            mCurrentFrame.mTcw = cv::Mat::zeros(0, 0, CV_32F); // this prevents a segfault later (https://github.com/raulmur/ORB_SLAM2/pull/381#issuecomment-337312336)
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }
    }

    void Tracking::Reset()
    {

        cout << "System Reseting" << endl;

        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;

        // Clear BoW Database
        cout << "Reseting Database...";
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        if (mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();
    }

    void Tracking::ChangeCalibration(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }


    //通过GMS进行目标的匹配
    void ObjectAssociationWithGMS(const Frame &LastFrame, const Frame &CurrentFrame, cv::Mat &out_img)
    {
        
    }

    // 使用光流法进行特征匹配
    void Tracking::TrackFeaturesOpticalFlowPyrLK(const Frame &imgrayLast, const cv::Mat &imgrayCur, 
        std::vector<int>& feature_matches, const cv::Mat& Tcl)
    {
        std::vector<cv::Point2f> prepoint, nextpoint;
        std::vector<uchar> forward_state;
        std::vector<float> forward_err;




        for(size_t i=0;i<imgrayLast.N;i++)
        {
            prepoint.push_back(imgrayLast.mvKeys.at(i).pt);
        }
    }


    cv::Mat Tracking::ComputeH12(const Frame&F1, const Frame &F2)
    {
        cv::Mat R1w = F1.mTcw.rowRange(0,3).colRange(0,3);
        cv::Mat t1w = F1.mTcw.rowRange(0,3).col(3);
        cv::Mat R2w = F2.mTcw.rowRange(0,3).colRange(0,3);
        cv::Mat t2w = F2.mTcw.rowRange(0,3).col(3);

        return cv::Mat::eye(3,3,CV_32F);
    }



    //通过光流法进行目标框的跟踪
    void Tracking::ProcessMovingObject(const cv::Mat &imgrayLast, const cv::Mat &imgrayCur, 
        const cv::Mat &dynamic_mask, cv::Mat& debug_image, bool flow_back)
    {
        // Clear the previous data
        std::vector<cv::Point2f> F_prepoint;
        std::vector<cv::Point2f> F_nextpoint;
        std::vector<cv::Point2f> F2_prepoint;
        std::vector<cv::Point2f> F2_nextpoint;
        
        std::vector<cv::Point2f> T_M;

        // Detect dynamic target and ultimately optput the T matrix
        std::vector<cv::Point2f> prepoint, nextpoint;
        std::vector<uchar> forward_state;
        std::vector<float> forward_err;
        cv::goodFeaturesToTrack(imgrayLast, prepoint, 1000, 0.01, 8, dynamic_mask, 3, true, 0.04);
        // cv::goodFeaturesToTrack(imgrayLast, prepoint, 1000, 0.001, 3, dynamic_mask);
        cv::cornerSubPix(imgrayLast, prepoint, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
        std::cout<<"begin OpticalFlowPyrLK"<<std::endl;
        // cv::calcOpticalFlowPyrLK(imgrayLast, imgrayCur, prepoint, nextpoint, forward_state, forward_err, cv::Size(22, 22), 5, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01),cv::OPTFLOW_USE_INITIAL_FLOW);
        cv::calcOpticalFlowPyrLK(imgrayLast, imgrayCur, prepoint, nextpoint, forward_state, forward_err, cv::Size(22, 22), 5, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
        std::cout<<"end OpticalFlowPyrLK"<<std::endl;
        
        const float limit_edge_corner = 5.0;
        const float limit_dis_epi     = 1.0;
        const float limit_of_check    = 2210;

        // reverse check
        if(flow_back)
        {
            std::cout<<"reverse optical flow check"<<std::endl;
            std::vector<uchar>  reverse_state(forward_state.size());
            std::vector<float> reverse_err;
            std::vector<cv::Point2f> reverse_pts, reverse_nextpoint;
            reverse_pts = nextpoint;
            cv::calcOpticalFlowPyrLK(imgrayCur, imgrayLast, reverse_pts, reverse_nextpoint, reverse_state, reverse_err, cv::Size(22, 22), 5, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
        
            std::cout<<"filter failed optical flow result"<<std::endl;
            for (int i = 0; i < forward_state.size(); i++)
            {
                cv::Point2f dif_pt = reverse_nextpoint[i] - prepoint[i];
                float dist = std::sqrt(dif_pt.x*dif_pt.x + dif_pt.y*dif_pt.y);
                if(forward_state[i] == 0 ||  reverse_state[i] == 0 ||  dist > limit_dis_epi)
                {
                    forward_state[i] = 0;
                    continue;
                }

                int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
                int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
                int x1 = prepoint[i].x,  y1 = prepoint[i].y;
                int x2 = nextpoint[i].x, y2 = nextpoint[i].y;
                if ((x1 < limit_edge_corner || x1 >= imgrayLast.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= imgrayLast.cols - limit_edge_corner
                    || y1 < limit_edge_corner || y1 >= imgrayLast.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= imgrayLast.rows - limit_edge_corner))
                {
                    forward_state[i] = 0;
                    continue;
                }


                //统计像素误差
                double sum_check = 0;
                for (int j = 0; j < 9; j++)
                    sum_check += abs(imgrayLast.at<uchar>(y1 + dy[j], x1 + dx[j]) - imgrayCur.at<uchar>(y2 + dy[j], x2 + dx[j]));
                
                if (sum_check > limit_of_check) 
                {
                    forward_state[i] = 0;
                    continue;
                }    

                forward_state[i] = 1;

                F_prepoint.push_back(prepoint[i]);
                F_nextpoint.push_back(nextpoint[i]);
            }
        }
        else
        {
            for (int i = 0; i < forward_state.size(); i++)
            {
                if(forward_state[i] != 0)
                {
                    int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
                    int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
                    int x1 = prepoint[i].x,  y1 = prepoint[i].y;
                    int x2 = nextpoint[i].x, y2 = nextpoint[i].y;
                    if ((x1 < limit_edge_corner || x1 >= imgrayLast.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= imgrayLast.cols - limit_edge_corner
                        || y1 < limit_edge_corner || y1 >= imgrayLast.rows - limit_edge_corner || y2 < limit_edge_corner || y2 >= imgrayLast.rows - limit_edge_corner))
                    {
                        forward_state[i] = 0;
                        continue;
                    }
                    //统计像素误差
                    double sum_check = 0;
                    for (int j = 0; j < 9; j++)
                        sum_check += abs(imgrayLast.at<uchar>(y1 + dy[j], x1 + dx[j]) - imgrayCur.at<uchar>(y2 + dy[j], x2 + dx[j]));
                    
                    if (sum_check > limit_of_check) 
                        forward_state[i] = 0;
                    
                    if (forward_state[i])
                    {
                        F_prepoint.push_back(prepoint[i]);
                        F_nextpoint.push_back(nextpoint[i]);
                    }
                }
            }
        }
        

        //显示光流结果
        int nCols = imgrayLast.cols;
        int nRows = imgrayLast.rows;
        cv::vconcat(imgrayLast,imgrayCur,debug_image);
        if(debug_image.channels() == 1)
            cv::cvtColor(debug_image,debug_image,CV_GRAY2BGR);

        for(size_t i=0;i<F_prepoint.size();i++)
        {
            cv::circle(debug_image, F_prepoint.at(i), 2.0, cv::Scalar(0, 255, 0));
            cv::circle(debug_image, F_nextpoint.at(i) + cv::Point2f(0,nRows), 2.0, cv::Scalar(255, 0, 0));
            cv::line(debug_image,F_prepoint.at(i),F_nextpoint.at(i) + cv::Point2f(0,nRows), cv::Scalar(0, 0, 255),1,CV_AA);
        }
        
    }



    // 目标关联
    void Tracking::ObjectAssociation(const Frame &LastFrame, const Frame &CurrentFrame, cv::Mat &out_img, bool use_epipolar_constraint)
    {
        std::vector<cv::Point2f> lastobjpts, currobjpts;
        std::vector<cv::Point2f> last_static_objpts, curr_static_objpts;
        for (size_t i = 0; i < LastFrame.mvKeysUn.size(); i++)
        {
            //归一化特征点
            // Eigen::Vector3d uv1 = mpPinholeCamera->cam2world(Eigen::Vector2d(LastFrame.mvKeysUn[i].pt.x,LastFrame.mvKeysUn[i].pt.y));
            // cv::Point2f pt_normal(uv1.x(),uv1.y());
            
            if (LastFrame.mvbKeysStatic[i])
            {
                // lastobjpts.push_back(pt_normal);
                lastobjpts.push_back(LastFrame.mvKeysUn[i].pt);
            }    
            else
            {
                // last_static_objpts.push_back(pt_normal);
                last_static_objpts.push_back(LastFrame.mvKeysUn[i].pt);
            }
                
        }

        cv::Mat last_image = LastFrame.mvImgPyramid[0].clone();
        cv::Mat cur_image  = CurrentFrame.mvImgPyramid[0].clone();
        int nRows = last_image.rows;
        int nCols = last_image.cols;

        out_img = cv::Mat(nRows, nCols * 2, last_image.type());

        std::cout<<"last_image.type:  "<<last_image.type()<<std::endl;

        last_image.copyTo(out_img.colRange(0, nCols));
        cur_image.copyTo(out_img.colRange(nCols, nCols * 2));

        if (out_img.channels() == 1)
            cv::cvtColor(out_img, out_img, CV_GRAY2BGR);

        last_image.convertTo(last_image, CV_8U, 255.0);
        cur_image.convertTo(cur_image, CV_8U, 255.0);

        cv::Mat F21;
        // 背景的光流
        if(use_epipolar_constraint)
        {
            std::vector<uchar> static_status;
            std::vector<float> static_err;
            cv::calcOpticalFlowPyrLK(last_image, cur_image, last_static_objpts, curr_static_objpts, static_status, static_err, cv::Size(21, 21), 3, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
            
            int static_count = 0;
            std::vector<cv::Point2f> F_prepoint, F_nextpoint;
            for (size_t i = 0; i < static_status.size(); i++)
            {
                if (static_status[i])
                {
                    //特征点在图像边缘，剔除
                    if(!CurrentFrame.mpPinholeCamera->isInFrame(
                        Eigen::Vector2i(last_static_objpts.at(i).x,last_static_objpts.at(i).y),5))
                    {
                        static_status[i] = 0;
                        continue;
                    }

                    F_prepoint.push_back(last_static_objpts.at(i));
                    F_nextpoint.push_back(curr_static_objpts.at(i));
                    static_count++;
                }
            }
            std::cout<<"inliner tracked pts: "<<static_count<<std::endl;


            //计算基础矩阵
            std::vector<uchar> inliersMask(F_prepoint.size());
            F21 = cv::findFundamentalMat(F_prepoint, F_nextpoint, inliersMask, cv::FM_RANSAC, 0.1, 0.99);
            // std::cout<<"fundamental matrix: "<<F<<std::endl;
            // std::vector<cv::Point2f> F2_prepoint, F2_nextpoint;
            // for (int i = 0; i < inliersMask.size(); i++)
            // {
            //     if (inliersMask.at(i) == 0)
            //         continue;
            //     else
            //     {
            //         // Circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
            //         double A = F.at<double>(0, 0)*F_prepoint[i].x + F.at<double>(0, 1)*F_prepoint[i].y + F.at<double>(0, 2);
            //         double B = F.at<double>(1, 0)*F_prepoint[i].x + F.at<double>(1, 1)*F_prepoint[i].y + F.at<double>(1, 2);
            //         double C = F.at<double>(2, 0)*F_prepoint[i].x + F.at<double>(2, 1)*F_prepoint[i].y + F.at<double>(2, 2);
            //         double dd = fabs(A*F_nextpoint[i].x + B*F_nextpoint[i].y + C) / sqrt(A*A + B*B);  //Epipolar constraints
            //         if (dd <= 0.1)
            //         {
            //             F2_prepoint.push_back(F_prepoint[i]);
            //             F2_nextpoint.push_back(F_nextpoint[i]);
            //         }
            //     }
            // }
            // std::cout<<"inliner static matches: "<<F2_prepoint.size()<<std::endl;
        }


        // 目标的光流
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(last_image, cur_image, lastobjpts, currobjpts, status, err, cv::Size(21, 21), 3, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));
        float v_min = 0.0;
        float v_max = 2.0;
        float dv = v_max - v_min;
        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i] != 0)
            {
                if(use_epipolar_constraint)
                {
                     // Circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
                    double A = F21.at<double>(0, 0)*lastobjpts[i].x + F21.at<double>(0, 1)*lastobjpts[i].y + F21.at<double>(0, 2);
                    double B = F21.at<double>(1, 0)*lastobjpts[i].x + F21.at<double>(1, 1)*lastobjpts[i].y + F21.at<double>(1, 2);
                    double C = F21.at<double>(2, 0)*lastobjpts[i].x + F21.at<double>(2, 1)*lastobjpts[i].y + F21.at<double>(2, 2);
                    double dd = fabs(A*currobjpts[i].x + B*currobjpts[i].y + C) / sqrt(A*A + B*B);  //Epipolar constraints

                    float v = dd;
                    float r = 1.0;
                    float g = 1.0;
                    float b = 1.0;
                    if (v < v_min)
                        v = v_min;
                    if (v > v_max)
                        v = v_max;

                    if (v < v_min + 0.25 * dv)
                    {
                        r = 0.0;
                        g = 4 * (v - v_min) / dv;
                    }
                    else if (v < (v_min + 0.5 * dv))
                    {
                        r = 0.0;
                        b = 1 + 4 * (v_min + 0.25 * dv - v) / dv;
                    }
                    else if (v < (v_min + 0.75 * dv))
                    {
                        r = 4 * (v - v_min - 0.5 * dv) / dv;
                        b = 0.0;
                    }
                    else
                    {
                        g = 1 + 4 * (v_min + 0.75 * dv - v) / dv;
                        b = 0.0;
                    }

                    //dynamic points
                    cv::circle(out_img, lastobjpts.at(i), 2.0, cv::Scalar(r, g, b));
                    cv::circle(out_img, cv::Point2f(currobjpts.at(i).x + nCols, currobjpts.at(i).y), 2.0, cv::Scalar(r, g, b));
                    cv::line(out_img,lastobjpts.at(i),cv::Point2f(currobjpts.at(i).x + nCols, currobjpts.at(i).y),cv::Scalar(r, g, b),1,CV_AA);
                }
            }
        }
    }

    int Tracking::SearchByTrackingHarris(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    {
    }

    int Tracking::SearchByTracking(const Frame &LastFrame, const Frame &CurrentFrame, const float th, const bool bMono)
    {
        std::vector<cv::Point2f> lastobjpts, currobjpts;

        for (size_t i = 0; i < LastFrame.mvKeysUn.size(); i++)
        {
            if (LastFrame.mvbKeysStatic[i])
                lastobjpts.push_back(LastFrame.mvKeysUn[i].pt);
        }

        std::vector<uchar> status;
        std::vector<float> err;
        cv::Mat last_image = 255 * LastFrame.mvImgPyramid[0].clone();
        cv::Mat cur_image = 255 * CurrentFrame.mvImgPyramid[0].clone();
        cv::cvtColor(last_image, last_image, CV_GRAY2BGR);
        cv::cvtColor(cur_image, cur_image, CV_GRAY2BGR);
        cv::calcOpticalFlowPyrLK(last_image, cur_image, lastobjpts, currobjpts, status, err, cv::Size(21, 21), 3);

        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i])
            {
            }
        }
    }

    void Tracking::DrawFeatureMatching(const Frame &frame1, const Frame &frame2, 
        const std::vector<cv::KeyPoint>& pts1, const std::vector<cv::KeyPoint>& pts2, 
        std::vector<cv::DMatch> &matches, cv::Mat &out_img)
    {
        cv::Mat image1 = 255*frame1.mvImgPyramid[0].clone();
        cv::Mat image2 = 255*frame2.mvImgPyramid[0].clone();

        int nRows = image1.rows;
        int nCols = image1.cols;

        cv::vconcat(image1,image2,out_img);
        if (out_img.channels() == 1)
            cv::cvtColor(out_img, out_img, CV_GRAY2BGR);

        for (size_t i = 0; i < matches.size(); i++)
        {
            cv::DMatch dm = matches[i];
            if (dm.queryIdx < 0)
                continue;

            cv::Point2f pt_1 = pts1[dm.queryIdx].pt;
            cv::Point2f pt_2 = pts2[dm.trainIdx].pt;

            cv::circle(out_img, pt_1, 2, cv::Scalar(255, 0, 0), -1);
            cv::circle(out_img, pt_2 + cv::Point2f(0, nRows), 2, cv::Scalar(255, 0, 0), -1);
            
            cv::line(out_img, pt_1, pt_2 + cv::Point2f(0, nRows), cv::Scalar(0, 255, 0), 1, CV_AA);
        }
    }


    void Tracking::ShowFeatureMatching(const Frame &frame1, const Frame &frame2, std::vector<cv::DMatch> &matches, cv::Mat &out_img)
    {
        cv::Mat image1 = frame1.mvImgPyramid[0].clone();
        cv::Mat image2 = frame2.mvImgPyramid[0].clone();

        int nRows = image1.rows;
        int nCols = image1.cols;

        out_img = cv::Mat(nRows, nCols * 2, image1.type());

        image1.copyTo(out_img.colRange(0, nCols));
        image2.copyTo(out_img.colRange(nCols, nCols * 2));

        if (out_img.channels() == 1)
            cv::cvtColor(out_img, out_img, CV_GRAY2BGR);

        ORBmatcher orb_matcher;

        //显示目标检测结果
        for (size_t i = 0; i < frame1.mvYoloObjects.size(); i++)
        {
            ObjBox_ obj_box = frame1.mvYoloObjects.at(i);
            if (obj_box.Class > 7)
                continue;
            cv::rectangle(out_img, cv::Rect(obj_box.x, obj_box.y, obj_box.w, obj_box.h), cv::Scalar(255, 0, 0), 2, CV_AA);
        }

        for (size_t i = 0; i < frame2.mvYoloObjects.size(); i++)
        {
            ObjBox_ obj_box = frame2.mvYoloObjects.at(i);
            if (obj_box.Class > 7)
                continue;
            cv::rectangle(out_img, cv::Rect(obj_box.x + nCols, obj_box.y, obj_box.w, obj_box.h), cv::Scalar(255, 0, 0), 2, CV_AA);
        }

        //计算基础矩阵
        cv::Mat F12 = ComputeF12(frame1.mTcw, frame2.mTcw, frame1.mK, frame2.mK);

        for (size_t i = 0; i < matches.size(); i++)
        {
            cv::Scalar color = RandomColor(i);

            cv::DMatch dm = matches[i];
            if (dm.queryIdx < 0)
                continue;

            cv::KeyPoint pt_1 = frame1.mvKeysUn[dm.queryIdx];
            cv::KeyPoint pt_2 = frame2.mvKeysUn[dm.trainIdx];

            //判断是否满足极线约束
            float dist = orb_matcher.GetDistEpipolarLine(pt_1, pt_2, F12);
            // bool inliner = orb_matcher.CheckDistEpipolarLine(pt_1, pt_2, F12, &frame2);
            if (dist < 0.50)
            {
                //内点
                if (frame1.mvbGroundKeysUn[dm.queryIdx])
                {
                    //地面点
                    cv::circle(out_img, pt_1.pt, 2, cv::Scalar(255, 0, 0), -1);
                    cv::circle(out_img, cv::Point2f(pt_2.pt.x + nCols, pt_2.pt.y), 2, cv::Scalar(255, 0, 0), -1);

                    cv::line(out_img, pt_1.pt, cv::Point2f(pt_2.pt.x + nCols, pt_2.pt.y), cv::Scalar(0, 255, 0), 1, CV_AA);
                }
                else
                {
                    cv::circle(out_img, pt_1.pt, 2, cv::Scalar(0, 0, 255), -1);
                    cv::circle(out_img, cv::Point2f(pt_2.pt.x + nCols, pt_2.pt.y), 2, cv::Scalar(0, 0, 255), -1);
                }
            }
            else
            {
                //外点
                if (frame1.mvbGroundKeysUn[dm.queryIdx])
                {
                    cv::circle(out_img, pt_1.pt, 2, cv::Scalar(0, 0, 0), -1);
                    cv::circle(out_img, cv::Point2f(pt_2.pt.x + nCols, pt_2.pt.y), 2, cv::Scalar(0, 0, 0), -1);
                }
                else
                {
                    cv::circle(out_img, pt_1.pt, 2, cv::Scalar(0, 255, 0), -1);
                    cv::circle(out_img, cv::Point2f(pt_2.pt.x + nCols, pt_2.pt.y), 2, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
    }

} //namespace ORB_SLAM
