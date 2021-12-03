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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>  //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件

namespace ORB_SLAM2
{

    long unsigned int Frame::nNextId = 0;
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;
    
    std::ofstream Frame::mEfficiency_fs("/home/kinggreat24/pc/efficiency.txt");
    
    Frame::Frame()
    {
    }

    //Copy Constructor
    Frame::Frame(const Frame &frame)
        : mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
          mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
          mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
          mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
          mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
          mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
          mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
          mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
          mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
          mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
          mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
          mpGroundPoints(frame.mpGroundPoints), mpObstaclePoints(frame.mpObstaclePoints),
          mpLidarPointCloudCamera(frame.mpLidarPointCloudCamera), mpMagLidarPointCloud(frame.mpMagLidarPointCloud),
          mpCornerPointsSharp(frame.mpCornerPointsSharp), mpCornerPointsLessSharp(frame.mpCornerPointsLessSharp),
          mpSurfPointsFlat(frame.mpSurfPointsFlat), mpSurfPointsLessFlat(frame.mpSurfPointsLessFlat),
          mvImgPyramid(frame.mvImgPyramid), mvYoloObjects(frame.mvYoloObjects), mpDT_ground(frame.mpDT_ground)
    {
        //全部原始激光点云
        mpLidarPointCloudRaw = frame.mpLidarPointCloudRaw;

        // 地面特征点
        mvbGroundKeysUn    = frame.mvbGroundKeysUn;
        mvbKeysStatic      = frame.mvbKeysStatic;

        // 目标点云
        mv_lidar_clusters_label = frame.mv_lidar_clusters_label;

        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j] = frame.mGrid[i][j];

        if (!frame.mTcw.empty())
            SetPose(frame.mTcw);
    }

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mpReferenceKF(static_cast<KeyFrame *>(NULL))
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoMatches();

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0, imGray);

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoFromRGBD(imDepth);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), mnTrackLevels(3)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0, imGray);

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }
        mb = mbf / fx;

        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imGray, const std::string &label_file, const std::string &lidar_file, const double &timeStamp,
                 ORBextractor *extractor, ORBVocabulary *voc, depth_clustering::SphericalProjection *pSphericalProjection, loam::MultiScanRegistration *pMultiRegistration, linefit_ground_segmentation::GroundSegmentationParams groundSegemtationParams, vk::PinholeCamera *pinhole_camera, cv::Mat &K, cv::Mat &distCoef, Eigen::Matrix4f &lidar2cam_extric, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mpPinholeCamera(pinhole_camera), mLidar2CameraExtric(lidar2cam_extric), mnTrackLevels(3),
          mpMultiScanRegistration(pMultiRegistration), mGroundSegmentationParams(groundSegemtationParams), mpSphericalProjection(pSphericalProjection)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0, imGray);
        ExtractLidar(lidar_file);

        
        //读取目标检测结果
        // LoadDetectionResults(label_file,mvYoloObjects);                    
        // std::cout<<"yolo object size: "<<mvYoloObjects.size()<<std::endl;

        // std::thread threadOrb(&Frame::ExtractORB,this,0,imGray);
        // std::thread threadLidar(&Frame::ExtractLidar,this,lidar_file);
        // threadOrb.join();
        // threadLidar.join();

        // cv::Mat image_with_features = imGray.clone();
        // ShowFeaturePoints(image_with_features);
        // char file_name[128]={0};
        // sprintf(file_name,"/home/kinggreat24/pc/%06d_features.png",mnId);
        // cv::imwrite(file_name,image_with_features);

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeBoW();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();


        // 提取目标检测框内的特征点
        mvbKeysStatic.resize(N,false);
        for(size_t j=0;j<mvKeys.size();j++)
        {
            cv::KeyPoint kp = mvKeys.at(j);
            if(IsInsideObjectBox(kp.pt))
                mvbKeysStatic[j] = true;
        }
        
        // 直接法采样
        PointSampling();
    }

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j].reserve(nReserve);

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    void Frame::ExtractORB(int flag, const cv::Mat &im)
    {
        if (flag == 0)
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);
        else
            (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);

        //图像金字塔
        cv::Mat original_img_ = im.clone();
        original_img_.convertTo(original_img_, CV_32FC1, 1.0 / 255);
        create_image_pyramid(original_img_, mnTrackLevels, mvImgPyramid);
    }

    void Frame::ExtractLidar(const std::string &lidar_file)
    {
        // (1)读取点云
        // pcl::PointCloud<pcl::PointXYZI>::Ptr curPointCloud(new pcl::PointCloud<pcl::PointXYZI>());
        mpLidarPointCloudRaw.reset(new pcl::PointCloud<pcl::PointXYZI>());
        ReadPointCloud(lidar_file, mpLidarPointCloudRaw);
        mpLidarPointCloudCamera.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*mpLidarPointCloudRaw, *mpLidarPointCloudCamera, mLidar2CameraExtric);

        // (2)地面分割
        pcl::PointCloud<pcl::PointXYZI>::Ptr pObstaclePoints(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pGroundPoints(new pcl::PointCloud<pcl::PointXYZI>());
        
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        GroundSegmentation(mpLidarPointCloudRaw,pObstaclePoints,pGroundPoints);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double t_time1= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        // std::cout<<"time of create ground terrian: "<<t_time<<std::endl;
        
        mpGroundPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mpObstaclePoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*pObstaclePoints, *mpObstaclePoints, mLidar2CameraExtric);
        pcl::transformPointCloud(*pGroundPoints, *mpGroundPoints, mLidar2CameraExtric);

        // (4)生成距离图
        depth_clustering::Cloud::Ptr pDepthClusteringCloud = depth_clustering::CloudFromPCL(pObstaclePoints);
        pDepthClusteringCloud->InitProjection(mpSphericalProjection->params());
        cv::Mat depth_image_raw  = pDepthClusteringCloud->projection_ptr()->depth_image();
        
        // 显示距离图
        // cv::Mat depth_color;
        // Gray2color(depth_image_raw,depth_color,0,80);
        // char file_name[128]={0};
        // sprintf(file_name,"/home/kinggreat24/%d_depth_image.png",mnId);
        // cv::imwrite(file_name,depth_color);

        // (5)点云分割
        cv::Mat mRangeImage = RepairDepth(depth_image_raw, 5, 1.0f);
        pDepthClusteringCloud->projection_ptr()->depth_image() = mRangeImage;
        int min_cluster_size = 30;
        int max_cluster_size = 40000;
        depth_clustering::Radians angle_tollerance = 10_deg;
        depth_clustering::ImageBasedClusterer<depth_clustering::LinearImageLabeler<>> image_clusterer(
            angle_tollerance, min_cluster_size, max_cluster_size);
        image_clusterer.SetDiffType(depth_clustering::DiffFactory::DiffType::ANGLES);
        image_clusterer.OnNewObjectReceived(*(pDepthClusteringCloud.get()), mvClusters);


        // (6)激光雷达特征提取
        mpCornerPointsSharp.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mpCornerPointsLessSharp.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mpSurfPointsFlat.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mpSurfPointsLessFlat.reset(new pcl::PointCloud<pcl::PointXYZI>());

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        mpMultiScanRegistration->handleCloudMessage(
            *mpObstaclePoints,
            std::chrono::high_resolution_clock::now(),
            *mpCornerPointsSharp,
            *mpCornerPointsLessSharp,
            *mpSurfPointsFlat,
            *mpSurfPointsLessFlat);
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        double t_time2= std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();
        // char static_pc_name[128]={0};
        // sprintf(static_pc_name,"/home/kinggreat24/pc/test/%d_SurfPointsLessFlat.pcd",mnId);
        // pcl::io::savePCDFileASCII(static_pc_name, *mpSurfPointsLessFlat);//将点云保存到PCD文件中
    
        //保存时间
        mEfficiency_fs << t_time1 <<"  "<<t_time2<<std::endl;
    }

    void Frame::ReadPointCloud(const string &lidar_file, pcl::PointCloud<pcl::PointXYZI>::Ptr &curPointCloud)
    {
        string suffix_str = lidar_file.substr(lidar_file.find_last_of('.') + 1);
        if (suffix_str == "bin")
        {
            // load point cloud
            std::fstream input(lidar_file.c_str(), std::ios::in | std::ios::binary);
            if (!input.good())
            {
                std::cerr << "Could not read file: " << lidar_file << std::endl;
                exit(EXIT_FAILURE);
            }

            for (int i = 0; input.good() && !input.eof(); i++)
            {
                pcl::PointXYZI point;
                float intensity = 0;
                input.read((char *)&point.x, 3 * sizeof(float));
                input.read((char *)&(point.intensity), sizeof(float));

                //remove all points behind image plane (approximation)
                /*if (point.x < mMinDepth)
                continue;*/
                curPointCloud->push_back(point);
            }
            input.close();
        }
        else
        {
            if (-1 == pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_file, *curPointCloud))
            {
                std::cerr << "Could not read file: " << lidar_file << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        curPointCloud->height = 1;
        curPointCloud->width = curPointCloud->size();
    }

    void Frame::GroundSegmentation(const pcl::PointCloud<pcl::PointXYZI>::Ptr &curPointCloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &pObstacleClouds,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &pGroundClouds)
    {
        // 地面提取
        pcl::PointCloud<pcl::PointXYZ> all_pointcloud;
        pcl::copyPointCloud(*curPointCloud, all_pointcloud);
        std::vector<int> vGroundLabels(all_pointcloud.size(), 0);
        linefit_ground_segmentation::GroundSegmentation ground_segmentation(mGroundSegmentationParams);
        // mpGroundSegemtation = new linefit_ground_segmentation::GroundSegmentation(mGroundSegmentationParams);
        ground_segmentation.segment(all_pointcloud, &vGroundLabels);

        for (int i = 0; i < vGroundLabels.size(); i++)
        {
            if (vGroundLabels[i] == 1)
                pGroundClouds->push_back(curPointCloud->at(i));
            else
                pObstacleClouds->push_back(curPointCloud->at(i));
        }

    #ifdef USE_RANSAC_PLANE
        // 对地面点进行ransac，计算法相之后，对激光雷达进行调平
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> plane_seg;

        // Optional
        plane_seg.setOptimizeCoefficients(true);

        // Mandatory
        plane_seg.setModelType(pcl::SACMODEL_PLANE);
        plane_seg.setMethodType(pcl::SAC_RANSAC);
        plane_seg.setDistanceThreshold(0.05);
        plane_seg.setInputCloud(pGroundClouds);
        plane_seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            return;
        }

        std::cerr << "Ground Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;
        std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

        Eigen::Vector3f normal_ground(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

        // 地面法相为正
        if (coefficients->values[2] < 0)
            normal_ground *= -1;

        // 调平变换矩阵
        Eigen::Matrix4f adjust_matrix = Converter::CreateRotateMatrix(normal_ground, Eigen::Vector3f(0, 0, 1));

        Eigen::Vector3f normal_vector = adjust_matrix.block<3, 3>(0, 0) * normal_ground;
        std::cout << "After adjust normal vector: " << normal_vector.transpose() << std::endl;

        // 对于剩余的点，如果到地面的距离小于阈值，也将其归入到地面中
        // 双边滤波，删选离地面点比较近的点
        // for(int i=0;i<tmpObstaclePointCloud->size();i++)
        // {
        //     pcl::PointXYZI pt = tmpObstaclePointCloud->at(i);
        //     if(pt.y > coefficients->values[3] + 0.5)
        //     {
        //         mpObstaclePointCloud->push_back(pt);
        //     }
        //     else
        //     {
        //         float dist = normal_ground.x() * pt.x + normal_ground.y() * pt.y + normal_ground.z() * pt.z + coefficients->values[3];
        //         if(std::fabs(dist) > 0.1)
        //             mpObstaclePointCloud->push_back(pt);
        //         else
        //             mpGroundPointCloud->push_back(pt);
        //     }
        // }
    #endif//USE_RANSAC_PLANE

    }


    void Frame::GetSemanticObjectsPointCloud()
    {
        // (1)计算每个cluster的图像区域
        for (auto iter_cluster = mvClusters.begin(); iter_cluster != mvClusters.end(); ++iter_cluster)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pClusters = iter_cluster->second.ToPcl(iter_cluster->first);
            pcl::PointCloud<pcl::PointXYZI>::Ptr clusters_pointcloud_camera(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*pClusters, *clusters_pointcloud_camera, mLidar2CameraExtric);

            // 所有的cluster点云
            mvClusterPointClouds[iter_cluster->first] = clusters_pointcloud_camera;
            
            int cluster_u_min = 99999;
            int cluster_u_max = 0;
            int cluster_v_min = 99999;
            int cluster_v_max = 0;
            cv::Scalar color = Frame::randomColor(cv::getTickCount());

            for (auto iter = clusters_pointcloud_camera->begin(); iter != clusters_pointcloud_camera->end(); 
                iter++)
            {
                Eigen::Vector3d xyz_ref(iter->x, iter->y, iter->z);

                if (iter->z <= 0)
                    continue;

                Eigen::Vector2d uv_ref;
                uv_ref = mpPinholeCamera->world2cam(xyz_ref);

                if (!mpPinholeCamera->isInFrame(uv_ref.cast<int>(), 0))
                    continue;
                
                const float u_ref_f = uv_ref(0);
                const float v_ref_f = uv_ref(1);
                const int u_ref_i = static_cast<int>(u_ref_f);
                const int v_ref_i = static_cast<int>(v_ref_f);

                if(u_ref_i > cluster_u_max)
                    cluster_u_max = u_ref_i;

                if(u_ref_i < cluster_u_min)
                    cluster_u_min = u_ref_i;

                if(v_ref_i > cluster_v_max)
                    cluster_v_max = v_ref_i;

                if(v_ref_i < cluster_v_min)
                    cluster_v_min = v_ref_i;
            }

            Box cluster_box;
            cluster_box.x = cluster_u_min;
            cluster_box.y = cluster_v_min;
            cluster_box.w = cluster_u_max - cluster_u_min;;
            cluster_box.h = cluster_v_max - cluster_v_min;
       
            mv_cluster_image_region[iter_cluster->first] = cluster_box;
        }


        // (2) 计算每个目标和激光点云簇的IOU
        for(size_t i=0;i<mvYoloObjects.size();i++)
        {
            ObjBox_ obj_box  = mvYoloObjects.at(i);
            
            // Yolo目标检测框
            Box yolo_obj_box_;
            yolo_obj_box_.x = obj_box.x;
            yolo_obj_box_.y = obj_box.y;
            yolo_obj_box_.w = obj_box.w;
            yolo_obj_box_.h = obj_box.h;

            // 计算IOU
            float max_box_iou_value = 0.0;
            int cluster_id          = 0;
            
            //记录每个与该目标相交的cluster
            std::map<int, float> cluster_iou_vec;
            std::map<int, float> cluster_obj_iou_vec;
            for(auto iter = mv_cluster_image_region.begin(); iter != mv_cluster_image_region.end(); iter++)
            {
                float box_iou_value =  box_iou(yolo_obj_box_, iter->second);
                if(box_iou_value>0)
                    cluster_iou_vec[iter->first] = box_iou_value;
                else
                    continue;

                // (a U b) / b
                float box_obj_iou_value =  obj_iou(yolo_obj_box_, iter->second);
                cluster_obj_iou_vec[iter->first] = box_obj_iou_value;

                if(box_iou_value > max_box_iou_value)
                {
                    cluster_id = iter->first;
                    max_box_iou_value = box_iou_value;
                }
            }
            
            //最大IOU超过一定阈值
            if(max_box_iou_value > 0.2)
            {
                mv_lidar_clusters_label[cluster_id] = i;
            }
        }

        std::cout<<"mv_lidar_clusters_label size: "<<mv_lidar_clusters_label.size()<<std::endl;
    }




    int Frame::GetGroundKeyPoints()
    {
        mpGroundKeyPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());

        mvbGroundKeysUn.resize(N,false);        //（初始化）所有的点都不是地面点
        int ground_pts = 0;

        for(size_t i=0;i<mvKeys.size();i++)
        {
            cv::Point2f pt = mvKeys.at(i).pt;

            if(IsInsideObjectBox(pt))
                continue;
                
            GEOM_FADE25D::Point2 pfeature(pt.x, mnMaxY - pt.y, 0);
            GEOM_FADE25D::Triangle2 *pTriangle = mpDT_ground->locate(pfeature);

            Eigen::Vector3d normalVector;
            Eigen::Vector3d AB, AC, BC;

            GEOM_FADE25D::Point2 *pA, *pB, *pC;
            //落在物体构成的三角网上
            if (pTriangle)
            {
                //只用当前三角网进行深度拟合
                pA = pTriangle->getCorner(0);
                pB = pTriangle->getCorner(1);
                pC = pTriangle->getCorner(2);

                AB = Eigen::Vector3d(pB->x() - pA->x(), pB->y() - pA->y(), pB->z() - pA->z());
                AC = Eigen::Vector3d(pC->x() - pA->x(), pC->y() - pA->y(), pC->z() - pA->z());
                BC = Eigen::Vector3d(pC->x() - pB->x(), pC->y() - pB->y(), pC->z() - pB->z());

                //三角网长度不能太长
                if (AB.x() > 30 || AB.y() > 30 || AC.x() > 30 || AC.y() > 30 || BC.x() > 30 || BC.y() > 30)
                    continue;
            }
            else
                continue;

            normalVector = AB.cross(AC);
            normalVector.normalize();

            Eigen::Vector3d AP(pfeature.x() - pA->x(), pfeature.y() - pA->y(), pfeature.z() - pA->z());
            float depth = -(normalVector(0) * AP(0) + normalVector(1) * AP(1)) / normalVector(2) + pA->z();
            
            if(depth < 0)
                continue;

            mvbGroundKeysUn[i] = true;
            mvDepth[i]         = depth;

            Eigen::Vector3d pt_world = mpPinholeCamera->cam2world(Eigen::Vector2d(pt.x,pt.y)) * depth;

            pcl::PointXYZI p_w;
            p_w.x = pt_world.x();
            p_w.y = pt_world.y();
            p_w.z = pt_world.z();
            p_w.intensity = i;
            mpGroundKeyPoints->push_back(p_w);

            ground_pts++;
        }

        std::cout<<"ground_pts: "<<ground_pts<<std::endl;

        // char file_name[128]={0};
        // sprintf(file_name,"/home/kinggreat24/pc/%d_ground_pt.pcd",mnId);
        // if(pGroundPoints->size() > 0)
        //     pcl::io::savePCDFileASCII(file_name, *pGroundPoints);//将点云保存到PCD文件中

        return ground_pts;
    }

    //显示地面特征点
    cv::Mat Frame::ShowGroundKeyPoints()
    {
        cv::Mat img_out = mvImgPyramid[0];
        if (img_out.channels() == 1)
        {
            cv::cvtColor(img_out, img_out, cv::COLOR_GRAY2BGR);
        }
       
        float v_min = 1.0;
        float v_max = 50.0;

        for(size_t i=0;i<mvKeysUn.size();i++)
        {
            if(!mvbGroundKeysUn[i])
                continue;
            
            cv::KeyPoint kp = mvKeysUn.at(i);

            float dv = v_max - v_min;
            float v = mvDepth[i];
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

            cv::circle(img_out, kp.pt, 2.0, cv::Scalar(r, g, b), -1);
        }

        return img_out;
    }


    void Frame::DrawSemanticObjectsPointCloud(cv::Mat &image_out, size_t num_level)
    {
        cv::Mat img = mvImgPyramid[num_level];
        if (img.channels() == 1)
        {
            cv::cvtColor(img, image_out, cv::COLOR_GRAY2BGR);
        }
        else
        {
            img.copyTo(image_out);
        }
        const float scale = 1.0f / (1 << num_level);

        //绘制目标检测结果
        char label_name[128]={0};
        for (auto iter = mvYoloObjects.begin(); iter != mvYoloObjects.end(); iter++)
        {
            cv::Point2f pt1(iter->x, iter->y);
            cv::Point2f pt2(iter->x + iter->w, iter->y + iter->h);
            cv::rectangle(image_out, pt1, pt2, cv::Scalar(255, 0, 0),2);
        
            sprintf(label_name,"%s, %f",iter->label.c_str(),iter->prob);
            cv::putText(image_out, label_name, pt1 + cv::Point2f(10,10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB(255, 0, 0), 1, CV_AA);
        }

        GetSemanticObjectsPointCloud();

        for(auto iter_region = mv_cluster_image_region.begin(); iter_region != mv_cluster_image_region.end(); iter_region++)
        {
            if(mv_lidar_clusters_label.find(iter_region->first) == mv_lidar_clusters_label.end())
                continue;

            cv::Scalar color = Frame::randomColor(cv::getTickCount());

            // 绘制激光雷达的区域
            Box box_cluster = mv_cluster_image_region[iter_region->first];
            cv::Point2f pt1(box_cluster.x, box_cluster.y);
            cv::Point2f pt2(box_cluster.x+box_cluster.w, box_cluster.y+box_cluster.h);
            cv::rectangle(image_out, pt1, pt2, cv::Scalar(0, 255, 0)/255.0, 2); 
            
            // 绘制目标点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr pObjCloud = mvClusterPointClouds[iter_region->first];
            for(auto iter_pt = pObjCloud->begin(); iter_pt != pObjCloud->end(); iter_pt++)
            {
                Eigen::Vector3d xyz_ref(iter_pt->x, iter_pt->y, iter_pt->z);

                if (iter_pt->z <= 0)
                    continue;

                Eigen::Vector2d uv_ref;
                uv_ref = mpPinholeCamera->world2cam(xyz_ref);

                if (!mpPinholeCamera->isInFrame(uv_ref.cast<int>(), 0))
                    continue;

                cv::circle(image_out, cv::Point(uv_ref.x(), uv_ref.y()), 1.0, cv::Scalar(color[0], color[1], color[2]) / 255.0, -1);
            }
        }
    }

    void Frame::DirectGeoPointExtract()
    {

    }

    cv::Mat Frame::RepairDepth(const cv::Mat &no_ground_image, int step, float depth_threshold)
    {
        cv::Mat inpainted_depth = no_ground_image.clone();
        for (int c = 0; c < inpainted_depth.cols; ++c)
        {
            for (int r = 0; r < inpainted_depth.rows; ++r)
            {
                float &curr_depth = inpainted_depth.at<float>(r, c);
                if (curr_depth < 0.001f)
                {
                    int counter = 0;
                    float sum = 0.0f;
                    for (int i = 1; i < step; ++i)
                    {
                        if (r - i < 0)
                        {
                            continue;
                        }
                        for (int j = 1; j < step; ++j)
                        {
                            if (r + j > inpainted_depth.rows - 1)
                            {
                                continue;
                            }
                            const float &prev = inpainted_depth.at<float>(r - i, c);
                            const float &next = inpainted_depth.at<float>(r + j, c);
                            if (prev > 0.001f && next > 0.001f &&
                                fabs(prev - next) < depth_threshold)
                            {
                                sum += prev + next;
                                counter += 2;
                            }
                        }
                    }
                    if (counter > 0)
                    {
                        curr_depth = sum / counter;
                    }
                }
            }
        }
        return inpainted_depth;
    }

    GEOM_FADE25D::Fade_2D *Frame::CreateTerrain(pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_cam)
    {
        GEOM_FADE25D::Fade_2D *pDt = new GEOM_FADE25D::Fade_2D();

        //将点云转到相机坐标空间
        std::vector<GEOM_FADE25D::Point2> vPoints;
        for (int i = 0; i < pc_cam->size(); i++)
        {
            pcl::PointXYZI pt = pc_cam->points.at(i);
            if (pt.z < 0)
                continue;

            //投影到图像上
            // float u = fx * pt.x / pt.z + cx;
            // float v = fy * pt.y / pt.z + cy;

            // if (u >= mnMaxX || u < mnMinX || v >= mnMaxY || v < mnMinY)
            //     continue;

            Eigen::Vector2d uv_ref = mpPinholeCamera->world2cam(Eigen::Vector3d(pt.x,pt.y,pt.z));
                if(!mpPinholeCamera->isInFrame(uv_ref.cast<int>(),0))
                    continue;
            
            float u = uv_ref.x();
            float v = uv_ref.y();

            GEOM_FADE25D::Point2 p(u, mnMaxY - v, pt.z);
            p.setCustomIndex(0);

            vPoints.push_back(p);
        }
        
        // GEOM_FADE25D::EfficientModel em(vPoints);
        // vPoints.clear();
        // double maxError(.1);
        // em.extract(maxError,vPoints);

        pDt->insert(vPoints);
        return pDt;
    }


    float Frame::DepthFitting(GEOM_FADE25D::Fade_2D *pdt, const cv::Point2f &pt)
    {
        GEOM_FADE25D::Point2 pfeature(pt.x, mnMaxY - pt.y, 0);
        GEOM_FADE25D::Triangle2 *pTriangle = pdt->locate(pfeature);

        Eigen::Vector3d normalVector;
        Eigen::Vector3d AB, AC, BC;

        GEOM_FADE25D::Point2 *pA, *pB, *pC;
        //落在物体构成的三角网上
        if (pTriangle)
        {
            //只用当前三角网进行深度拟合
            pA = pTriangle->getCorner(0);
            pB = pTriangle->getCorner(1);
            pC = pTriangle->getCorner(2);

            AB = Eigen::Vector3d(pB->x() - pA->x(), pB->y() - pA->y(), pB->z() - pA->z());
            AC = Eigen::Vector3d(pC->x() - pA->x(), pC->y() - pA->y(), pC->z() - pA->z());
            BC = Eigen::Vector3d(pC->x() - pB->x(), pC->y() - pB->y(), pC->z() - pB->z());

            //三角网长度不能太长
            if (AB.x() > 30 || AB.y() > 30 || AC.x() > 30 || AC.y() > 30 || BC.x() > 30 || BC.y() > 30)
                return -1.0;
        }
        else
            return -1.0;

        normalVector = AB.cross(AC);
        normalVector.normalize();

        Eigen::Vector3d AP(pfeature.x() - pA->x(), pfeature.y() - pA->y(), pfeature.z() - pA->z());
        float depth = -(normalVector(0) * AP(0) + normalVector(1) * AP(1)) / normalVector(2) + pA->z();
        return depth;
    }

    void Frame::ShowClusterPointClouds(const std::unordered_map<uint16_t, depth_clustering::Cloud> &vLidarClusters,
        cv::Mat &image_out, size_t num_level)
    {
        cv::Mat img = mvImgPyramid[num_level];
        if (img.channels() == 1)
        {
            cv::cvtColor(img, image_out, cv::COLOR_GRAY2BGR);
        }
        else
        {
            img.copyTo(image_out);
        }

        const float scale = 1.0f / (1 << num_level);

        for (auto iter_cluster = vLidarClusters.begin(); iter_cluster != vLidarClusters.end(); ++iter_cluster)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pClusters = iter_cluster->second.ToPcl(iter_cluster->first);

            pcl::PointCloud<pcl::PointXYZI>::Ptr clusters_pointcloud_camera(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*pClusters, *clusters_pointcloud_camera, mLidar2CameraExtric);

            cv::Scalar color = Frame::randomColor(cv::getTickCount());
            
            //cluster的图像区域
            int cluster_u_min = 99999;
            int cluster_u_max = 0;
            int cluster_v_min = 99999;
            int cluster_v_max = 0;

            for (auto iter = clusters_pointcloud_camera->begin(); iter != clusters_pointcloud_camera->end(); iter++)
            {
                Eigen::Vector3d xyz_ref(iter->x, iter->y, iter->z);

                if (iter->z <= 0)
                    continue;

                Eigen::Vector2d uv_ref;
                uv_ref = mpPinholeCamera->world2cam(xyz_ref) * scale;

                if (!mpPinholeCamera->isInFrame(uv_ref.cast<int>(), 0))
                    continue;

                const float u_ref_f = uv_ref(0);
                const float v_ref_f = uv_ref(1);
                const int u_ref_i = static_cast<int>(u_ref_f);
                const int v_ref_i = static_cast<int>(v_ref_f);

                if(u_ref_i > cluster_u_max)
                    cluster_u_max = u_ref_i;

                if(u_ref_i < cluster_u_min)
                    cluster_u_min = u_ref_i;

                if(v_ref_i > cluster_v_max)
                    cluster_v_max = v_ref_i;

                if(v_ref_i < cluster_v_min)
                    cluster_v_min = v_ref_i;

                cv::circle(image_out, cv::Point(u_ref_i, v_ref_i), 1.0, cv::Scalar(color[0], color[1], color[2]) / 255.0, -1);
            }
            cv::Point2f pt1(cluster_u_min, cluster_v_min);
            cv::Point2f pt2(cluster_u_max, cluster_v_max);
            cv::rectangle(image_out, pt1, pt2, cv::Scalar(0, 255, 0)/255.0,2); 
        }
        image_out.convertTo(image_out, CV_8UC3, 255);
    }

    void Frame::LoadDetectionResults(const string &strPathToSequence, std::vector<ObjBox_> &yolo_objects)
    {
        //
        FILE *fp = fopen(strPathToSequence.c_str(), "r"); // 非 Windows 平台使用 "r"
        char readBuffer[65536];
        FileReadStream is(fp, readBuffer, sizeof(readBuffer));

        Document dom;
        dom.ParseStream(is);

        if (dom.HasMember("objects") && dom["objects"].IsArray())
        {
            const rapidjson::Value &objects = dom["objects"];

            for (int i = 0; i < objects.Size(); ++i)
            {
                const rapidjson::Value &obj = objects[i];

                ObjBox_ obj_box;

                //label
                obj_box.label = obj["label"].GetString();
                obj_box.prob = obj["prob"].GetFloat();

                // Bounding Box
                const rapidjson::Value &bounding_box = obj["bounding_box"];
                obj_box.x = bounding_box["x"].GetFloat();
                obj_box.y = bounding_box["y"].GetFloat();
                obj_box.w = bounding_box["w"].GetFloat();
                obj_box.h = bounding_box["h"].GetFloat();

                // Class
                obj_box.Class = obj["class"].GetInt();

                //只保留 {-person -bicycle -car -motorbike -aeroplane -bus -train -truck} 
                if(obj_box.Class > 7)
                    continue;

                yolo_objects.push_back(obj_box);
            }
        }
        fclose(fp);
    }

    cv::Mat Frame::GenerateObjectMask()
    {
        //生成一个
        // std::cout<<"Frame::GenerateObjectMask begin, height: "<<mpPinholeCamera->height()<<" width: "<<mpPinholeCamera->width()<<std::endl;
        cv::Mat object_mask = cv::Mat::zeros(mnMaxY,mnMaxX,CV_8UC1);

        std::cout<<" debug 1 "<<std::endl;
        for(size_t i=0;i<mvYoloObjects.size();i++)
        {
            ObjBox_ yolo_obj_box = mvYoloObjects.at(i);
            cv::Mat mask = cv::Mat::ones(yolo_obj_box.h, yolo_obj_box.w, CV_8UC1);
            mask.copyTo(object_mask.colRange(yolo_obj_box.x,yolo_obj_box.x+yolo_obj_box.w).rowRange(yolo_obj_box.y,yolo_obj_box.y+yolo_obj_box.h));
        }
        std::cout<<"Frame::GenerateObjectMask over"<<std::endl;
        return object_mask;
    }


    void Frame::PointSampling()
    {
        int num_bucket_size = 40;

        vector<pair<float, PointType> > mag_point_bucket;
        mag_point_bucket.reserve(num_bucket_size);

        int num_out_points = 0;

        mpMagLidarPointCloud.reset(new pcl::PointCloud<PointType>());

        for (auto iter = mpLidarPointCloudCamera->begin(); iter != mpLidarPointCloudCamera->end(); iter++)
        {
            pcl::PointXYZI pt = *iter;
            if (pt.z < 0)
                continue;

            Eigen::Vector2d uv = mpPinholeCamera->world2cam(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (mpPinholeCamera->isInFrame(uv.cast<int>(), 0))
            {
                int u = static_cast<int>(uv(0));
                int v = static_cast<int>(uv(1));

                cv::Mat img = level(0);
                float dx = 0.5f * (img.at<float>(v, u + 1) - img.at<float>(v, u - 1));
                float dy = 0.5f * (img.at<float>(v + 1, u) - img.at<float>(v - 1, u));

                std::pair<float, PointType> mag_point;
                mag_point = make_pair((dx * dx + dy * dy), (*iter));

                mag_point_bucket.push_back(mag_point);
                if (mag_point_bucket.size() == num_bucket_size)
                {

                    float max = -1;
                    int idx;
                    for (int i = 0; i < mag_point_bucket.size(); ++i)
                    {
                        if (mag_point_bucket[i].first > max)
                        {
                            max = mag_point_bucket[i].first;
                            idx = i;
                        }
                    }

                    if (max > (6.25 / (255.0 * 255.0))) // 16.25
                        mpMagLidarPointCloud->push_back(mag_point_bucket[idx].second);

                    mag_point_bucket.clear();
                }
            }
        }
    }

    void Frame::SemanticPointSampling(const cv::Mat& dynamic_mask)
    {
        int num_bucket_size = 40;

        vector<pair<float, PointType> > mag_point_bucket;
        mag_point_bucket.reserve(num_bucket_size);

        int num_out_points = 0;

        mpMagLidarPointCloud.reset(new pcl::PointCloud<PointType>());

        for (auto iter = mpLidarPointCloudCamera->begin(); iter != mpLidarPointCloudCamera->end(); iter++)
        {
            pcl::PointXYZI pt = *iter;
            if (pt.z < 0)
                continue;

            Eigen::Vector2d uv = mpPinholeCamera->world2cam(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (mpPinholeCamera->isInFrame(uv.cast<int>(), 0))
            {
                int u = static_cast<int>(uv(0));
                int v = static_cast<int>(uv(1));

                if(dynamic_mask.at<cv::Vec3b>(v,u)[0] > 0 )
                    continue;

                cv::Mat img = level(0);
                float dx = 0.5f * (img.at<float>(v, u + 1) - img.at<float>(v, u - 1));
                float dy = 0.5f * (img.at<float>(v + 1, u) - img.at<float>(v - 1, u));

                std::pair<float, PointType> mag_point;
                mag_point = make_pair((dx * dx + dy * dy), (*iter));

                mag_point_bucket.push_back(mag_point);
                if (mag_point_bucket.size() == num_bucket_size)
                {

                    float max = -1;
                    int idx;
                    for (int i = 0; i < mag_point_bucket.size(); ++i)
                    {
                        if (mag_point_bucket[i].first > max)
                        {
                            max = mag_point_bucket[i].first;
                            idx = i;
                        }
                    }

                    // 梯度
                    if (max > (6.25 / (255.0 * 255.0))) // 16.25
                        mpMagLidarPointCloud->push_back(mag_point_bucket[idx].second);

                    mag_point_bucket.clear();
                }
            }
        }
    }


    //利用更加稳定深度的激光点进行计算(平坦点与地面点)
    void Frame::RobustPointSampling()
    {
        int num_surf_bucket_size = 10;
        int num_ground_bucket_size = 20;

        vector<pair<float, PointType> > surf_mag_point_bucket;
        surf_mag_point_bucket.reserve(num_surf_bucket_size);

        vector<pair<float, PointType> > ground_mag_point_bucket;
        ground_mag_point_bucket.reserve(num_ground_bucket_size);

        int num_out_points = 0;

        mpMagLidarPointCloud.reset(new pcl::PointCloud<PointType>());

        for (auto iter = mpSurfPointsLessFlat->begin(); iter != mpSurfPointsLessFlat->end(); iter++)
        {
            pcl::PointXYZI pt = *iter;
            if (pt.z < 0)
                continue;

            Eigen::Vector2d uv = mpPinholeCamera->world2cam(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (mpPinholeCamera->isInFrame(uv.cast<int>(), 4))
            {
                int u = static_cast<int>(uv(0));
                int v = static_cast<int>(uv(1));

                cv::Mat img = level(0);
                float dx = 0.5f * (img.at<float>(v, u + 1) - img.at<float>(v, u - 1));
                float dy = 0.5f * (img.at<float>(v + 1, u) - img.at<float>(v - 1, u));

                std::pair<float, PointType> mag_point;
                mag_point = make_pair((dx * dx + dy * dy), (*iter));

                surf_mag_point_bucket.push_back(mag_point);
                if (surf_mag_point_bucket.size() == num_surf_bucket_size)
                {

                    float max = -1;
                    int idx;
                    for (int i = 0; i < surf_mag_point_bucket.size(); ++i)
                    {
                        if (surf_mag_point_bucket[i].first > max)
                        {
                            max = surf_mag_point_bucket[i].first;
                            idx = i;
                        }
                    }

                    if (max > (6.25 / (255.0 * 255.0))) // 16.25
                        mpMagLidarPointCloud->push_back(surf_mag_point_bucket[idx].second);

                    surf_mag_point_bucket.clear();
                }
            }
        }


        //地面点采样
        for (auto iter = mpGroundPoints->begin(); iter != mpGroundPoints->end(); iter++)
        {
            pcl::PointXYZI pt = *iter;
            if (pt.z < 0)
                continue;

            Eigen::Vector2d uv = mpPinholeCamera->world2cam(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (mpPinholeCamera->isInFrame(uv.cast<int>(), 4))
            {
                int u = static_cast<int>(uv(0));
                int v = static_cast<int>(uv(1));

                cv::Mat img = level(0);
                float dx = 0.5f * (img.at<float>(v, u + 1) - img.at<float>(v, u - 1));
                float dy = 0.5f * (img.at<float>(v + 1, u) - img.at<float>(v - 1, u));

                std::pair<float, PointType> mag_point;
                mag_point = make_pair((dx * dx + dy * dy), (*iter));

                ground_mag_point_bucket.push_back(mag_point);
                if (ground_mag_point_bucket.size() == num_surf_bucket_size)
                {

                    float max = -1;
                    int idx;
                    for (int i = 0; i < ground_mag_point_bucket.size(); ++i)
                    {
                        if (ground_mag_point_bucket[i].first > max)
                        {
                            max = ground_mag_point_bucket[i].first;
                            idx = i;
                        }
                    }

                    if (max > (6.25 / (255.0 * 255.0))) // 16.25
                        mpMagLidarPointCloud->push_back(ground_mag_point_bucket[idx].second);

                    ground_mag_point_bucket.clear();
                }
            }
        }
    }

    void Frame::ShowPointClouds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &mpLidarPointCloud,
                                cv::Mat &image_out, size_t num_level)
    {
        cv::Mat img = mvImgPyramid[num_level];
        if (img.channels() == 1)
        {
            cvtColor(img, image_out, cv::COLOR_GRAY2BGR);
        }
        else
        {
            img.copyTo(image_out);
        }

        const float scale = 1.0f / (1 << num_level);

        int n = 0;
        for (auto iter = mpLidarPointCloud->begin(); iter != mpLidarPointCloud->end(); ++iter)
        {
            Eigen::Vector3d xyz_ref(iter->x, iter->y, iter->z);

            if (iter->z <= 0)
                continue;

            Eigen::Vector2d uv_ref;
            uv_ref = mpPinholeCamera->world2cam(xyz_ref) * scale;

            if (!mpPinholeCamera->isInFrame(uv_ref.cast<int>(), 0))
                continue;

            const float u_ref_f = uv_ref(0);
            const float v_ref_f = uv_ref(1);
            const int u_ref_i = static_cast<int>(u_ref_f);
            const int v_ref_i = static_cast<int>(v_ref_f);

            float v_min = 1.0;
            float v_max = 50.0;
            float dv = v_max - v_min;
            float v = xyz_ref[2];
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

            cv::circle(image_out, cv::Point(u_ref_i, v_ref_i), 1.0, cv::Scalar(r, g, b), -1);
        }
        image_out.convertTo(image_out, CV_8UC3, 255);
    }

    void Frame::ShowFeaturePoints(cv::Mat &image_out)
    {
        if(image_out.channels() == 1)
            cv::cvtColor(image_out,image_out,CV_GRAY2BGR);

        const float r = 5.0;
        for(size_t i=0;i<N;i++)
        {
            cv::Point2f pt1,pt2;
            pt1.x=mvKeys[i].pt.x-r;
            pt1.y=mvKeys[i].pt.y-r;
            pt2.x=mvKeys[i].pt.x+r;
            pt2.y=mvKeys[i].pt.y+r;
        
            cv::rectangle(image_out,pt1,pt2,cv::Scalar(0,255,0));
            cv::circle(image_out,mvKeys[i].pt,2,cv::Scalar(0,255,0),-1);
        }
    }


    cv::Scalar Frame::randomColor(int64 seed)
    {
        cv::RNG rng(seed);
        int icolor = (unsigned int)rng;
        return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
    }

    cv::Mat Frame::ShowDynamicKeyPoints()
    {
        const int num_level = 0;
        cv::Mat image_out = mvImgPyramid[num_level];
        if (image_out.channels() == 1)
        {
            cvtColor(image_out, image_out, cv::COLOR_GRAY2BGR);
        }
        
        //
        //显示目标检测结果
        for(size_t i=0;i<mvYoloObjects.size();i++)
        {
            ObjBox_ obj_box = mvYoloObjects.at(i);
            cv::rectangle(image_out,cv::Rect(obj_box.x, obj_box.y, obj_box.w, obj_box.h),cv::Scalar(255,0,0),2,CV_AA);
        }

        for(size_t i=0;i<mvKeysUn.size();i++)
        {
            cv::KeyPoint kp = mvKeysUn[i];
            if(mvbKeysStatic[i])
                cv::circle(image_out,kp.pt,2,cv::Scalar(0,0,255),-1);
        }

        return image_out;
    }

    void Frame::Gray2color(const cv::Mat &imgGray, cv::Mat &imgColor, const float v_min, const float v_max)
    {
        imgColor = cv::Mat(imgGray.rows, imgGray.cols, CV_8UC3);
        float dv = v_max - v_min;
        for (int i = 0; i < imgGray.rows; i++)
        {
            for (int j = 0; j < imgGray.cols; j++)
            {
                float v = imgGray.at<float>(i, j);
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

                imgColor.at<cv::Vec3b>(i, j)[0] = 255 * b;
                imgColor.at<cv::Vec3b>(i, j)[1] = 255 * g;
                imgColor.at<cv::Vec3b>(i, j)[2] = 255 * r;
            }
        }
    }

    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0, 3).col(3);
        mOw = -mRcw.t() * mtcw;
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw * P + mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if (PcZ < 0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        const float u = fx * PcX * invz + cx;
        const float v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P - mOw;
        const float dist = cv::norm(PO);

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf * invz;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }

    void Frame::ComputeBoW()
    {
        if (mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void Frame::UndistortKeyPoints()
    {
        if (mDistCoef.at<float>(0) == 0.0)
        {
            mvKeysUn = mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N, 2, CV_32F);
        for (int i = 0; i < N; i++)
        {
            mat.at<float>(i, 0) = mvKeys[i].pt.x;
            mat.at<float>(i, 1) = mvKeys[i].pt.y;
        }

        // Undistort points
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for (int i = 0; i < N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKeysUn[i] = kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if (mDistCoef.at<float>(0) != 0.0)
        {
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = imLeft.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = imLeft.rows;
            mat.at<float>(3, 0) = imLeft.cols;
            mat.at<float>(3, 1) = imLeft.rows;

            // Undistort corners
            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
            mat = mat.reshape(1);

            mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    void Frame::ComputeStereoMatches()
    {
        mvuRight = vector<float>(N, -1.0f);
        mvDepth = vector<float>(N, -1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        //Assign keypoints to row table
        vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for (int iR = 0; iR < Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf / minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int>> vDistIdx;
        vDistIdx.reserve(N);

        for (int iL = 0; iL < N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty())
                continue;

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for (size_t iC = 0; iC < vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                    continue;

                const float &uR = kpR.pt.x;

                if (uR >= minU && uR <= maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if (bestDist < thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x * scaleFactor);
                const float scaledvL = round(kpL.pt.y * scaleFactor);
                const float scaleduR0 = round(uR0 * scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
                IL.convertTo(IL, CV_32F);
                IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2 * L + 1);

                const float iniu = scaleduR0 + L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for (int incR = -L; incR <= +L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                    IR.convertTo(IR, CV_32F);
                    IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                if (bestincR == -L || bestincR == L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L + bestincR - 1];
                const float dist2 = vDists[L + bestincR];
                const float dist3 = vDists[L + bestincR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                if (deltaR < -1 || deltaR > 1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

                float disparity = (uL - bestuR);

                if (disparity >= minD && disparity < maxD)
                {
                    if (disparity <= 0)
                    {
                        disparity = 0.01;
                        bestuR = uL - 0.01;
                    }
                    mvDepth[iL] = mbf / disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int, int>(bestDist, iL));
                }
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--)
        {
            if (vDistIdx[i].first < thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second] = -1;
                mvDepth[vDistIdx[i].second] = -1;
            }
        }
    }

    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;

            const float d = imDepth.at<float>(v, u);

            if (d > 0)
            {
                mvDepth[i] = d;
                mvuRight[i] = kpU.pt.x - mbf / d;
            }
        }
    }

    cv::Mat Frame::UnprojectStereo(const int &i)
    {
        const float z = mvDepth[i];
        if (z > 0)
        {
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
            return mRwc * x3Dc + mOw;
        }
        else
            return cv::Mat();
    }

    template <typename T>
    static void pyrDownMeanSmooth(const cv::Mat &in, cv::Mat &out)
    {
        out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());

        // #pragma omp parallel for collapse(2)
        for (int y = 0; y < out.rows; ++y)
        {
            for (int x = 0; x < out.cols; ++x)
            {
                int x0 = x * 2;
                int x1 = x0 + 1;
                int y0 = y * 2;
                int y1 = y0 + 1;

                out.at<T>(y, x) = (T)((in.at<T>(y0, x0) + in.at<T>(y0, x1) + in.at<T>(y1, x0) + in.at<T>(y1, x1)) / 4.0f);
            }
        }
    }

    void create_image_pyramid(const cv::Mat &img_level_0, int n_levels, std::vector<cv::Mat> &pyramid)
    {
        pyramid.resize(n_levels);
        pyramid[0] = img_level_0;

        for (int i = 1; i < n_levels; ++i)
        {
            pyramid[i] = cv::Mat(pyramid[i - 1].rows / 2, pyramid[i - 1].cols / 2, CV_32FC1);
            pyrDownMeanSmooth<float>(pyramid[i - 1], pyramid[i]);
        }
    }

} //namespace ORB_SLAM
