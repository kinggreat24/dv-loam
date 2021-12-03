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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "Common.h"

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "loam/MultiScanRegistration.h"

#include "ground_segmentation/ground_segmentation.h"

#include "depth_clustering/projections/spherical_projection.h"
#include "depth_clustering/utils/cloud.h"
#include "depth_clustering/utils/velodyne_utils.h"
#include "clusterers/image_based_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"

//三角网
#include "include_fade25d/Fade_2D.h"

#include <opencv2/opencv.hpp>

#include <vikit/pinhole_camera.h>


//JSON
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h" // for stringify JSON
#include "rapidjson/filereadstream.h"
using namespace rapidjson;

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

void create_image_pyramid(const cv::Mat &img_level_0, int n_levels, std::vector<cv::Mat> &pyramid);

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras & Lidar.
    Frame(const cv::Mat &imGray, const std::string& label_file, const std::string& lidar_file, const double &timeStamp,
        ORBextractor* extractor,ORBVocabulary* voc, depth_clustering::SphericalProjection *pSphericalProjection, loam::MultiScanRegistration* pMultiRegistration,  linefit_ground_segmentation::GroundSegmentationParams groundSegemtationParams, vk::PinholeCamera* pinhole_camera, cv::Mat &K, cv::Mat &distCoef, Eigen::Matrix4f& lidar2cam_extric, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    void ExtractLidar(const std::string &lidar_file);

    void GroundSegmentation(const pcl::PointCloud<pcl::PointXYZI>::Ptr& curPointCloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &pObstacleClouds,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &pGroundClouds);

    //激光点采样
    void PointSampling();
    void SemanticPointSampling(const cv::Mat& dynamic_mask);
    void RobustPointSampling();

    //读取目标检测结果
    void LoadDetectionResults(const string &strPathToSequence, std::vector<ObjBox_>& yolo_objects);
    
    cv::Mat GenerateObjectMask();

    inline bool IsInsideObjectBox(const cv::Point2f& pt)
    {
        bool flag = false;
        for(size_t i=0;i<mvYoloObjects.size();i++)
        {
            ObjBox_ obj_box = mvYoloObjects[i];
            if(obj_box.Class >7)
                continue;

            if(pt.x > obj_box.x && pt.x < obj_box.x + obj_box.w && pt.y > obj_box.y && pt.y < obj_box.y+obj_box.h)
            {
                flag = true;
                break;
            }    
        }
        return flag;
    }

    inline int IsKeyPointInsideObjectBox(const cv::Point2f& pt)
    {
        int id = -1;
        for(size_t i=0;i<mvYoloObjects.size();i++)
        {
            ObjBox_ obj_box = mvYoloObjects[i];
            if(pt.x > obj_box.x && pt.x < obj_box.x + obj_box.w && 
                pt.y > obj_box.y && pt.y < obj_box.y+obj_box.h)
            {
                id = i;
                break;
            }    
        }
        return id;
    }
    

    void ReadPointCloud(const string &strPathToLidar, pcl::PointCloud<pcl::PointXYZI>::Ptr& curPointCloud);
    cv::Mat RepairDepth(const cv::Mat &no_ground_image, int step, float depth_threshold);

    void DirectGeoPointExtract();

    void GetSemanticObjectsPointCloud();
    void DrawSemanticObjectsPointCloud(cv::Mat &image_out, size_t num_level);


    //三角网相关方法
    GEOM_FADE25D::Fade_2D *CreateTerrain(pcl::PointCloud<pcl::PointXYZI>::Ptr &pGroundPointCloud);
    float DepthFitting(GEOM_FADE25D::Fade_2D *pdt, const cv::Point2f& pt);
    

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Get the image in pyramid
    cv::Mat& level(const size_t idx)
    {
        if(idx < mnTrackLevels)
            return mvImgPyramid[idx];
    }

    pcl::PointCloud<pcl::PointXYZI>& pointcloud()
    {
        return (*mpMagLidarPointCloud);
    }

    //空间点对姿态的导数
    inline static void jacobian_xyz2uv(const Eigen::Vector3f& xyz_in_f, Matrix2x6& J)
    {
        const float x = xyz_in_f[0];
        const float y = xyz_in_f[1];
        const float z_inv = 1./xyz_in_f[2];
        const float z_inv_2 = z_inv*z_inv;

        J(0,0) = -z_inv;              // -1/z
        J(0,1) = 0.0;                 // 0
        J(0,2) = x*z_inv_2;           // x/z^2
        J(0,3) = y*J(0,2);            // x*y/z^2
        J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
        J(0,5) = y*z_inv;             // y/z

        J(1,0) = 0.0;                 // 0
        J(1,1) = -z_inv;              // -1/z
        J(1,2) = y*z_inv_2;           // y/z^2
        J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
        J(1,4) = -J(0,3);             // -x*y/z^2
        J(1,5) = -x*z_inv;            // x/z
    }


    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    //　获取地面特征点以及线段
    int GetGroundKeyPoints();
    cv::Mat ShowGroundKeyPoints();


    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

    void Gray2color(const cv::Mat &imgGray, cv::Mat &imgColor, const float v_min, const float v_max);

    cv::Scalar randomColor(int64 seed);

    cv::Mat ShowDynamicKeyPoints();

    void ShowPointClouds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &mpLidarPointCloud, 
        cv::Mat &image_out, size_t num_level);
    
    void ShowClusterPointClouds(const std::unordered_map<uint16_t, depth_clustering::Cloud> &vLidarClusters,
        cv::Mat &image_out, size_t num_level);
    
    void ShowFeaturePoints(cv::Mat &image_out);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;
    vk::PinholeCamera* mpPinholeCamera;


    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    std::vector<bool> mvbGroundKeysUn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpGroundKeyPoints;      //地面特征点点云 

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    //直接法相关
    int mnTrackLevels;
    std::vector<cv::Mat> mvImgPyramid;

    //直接法采样点
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpLidarPointCloudRaw;         //原始雷达点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpLidarPointCloudCamera;      //相机坐标系下的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpMagLidarPointCloud;         //直接法采样点

    // Lidar Mapping相关
    depth_clustering::SphericalProjection *mpSphericalProjection;

    // Lidar ground extraction
    linefit_ground_segmentation::GroundSegmentationParams mGroundSegmentationParams;
    linefit_ground_segmentation::GroundSegmentation* mpGroundSegemtation;

    // Lidar feature extractor
    loam::MultiScanRegistration* mpMultiScanRegistration;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mpGroundPoints;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpObstaclePoints;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mpCornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpCornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpSurfPointsFlat;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mpSurfPointsLessFlat;

        
    std::unordered_map<uint16_t, depth_clustering::Cloud> mvClusters;      // 激光点云簇分割
    std::unordered_map<uint16_t, pcl::PointCloud<pcl::PointXYZI>::Ptr> mvClusterPointClouds;
    std::unordered_map<uint16_t, Box> mv_cluster_image_region;             // 激光点云簇在图像上的包围框
    std::unordered_map<uint16_t, int> mv_lidar_clusters_label;             // 点云簇对应的目标类别

    std::vector<ObjBox_> mvYoloObjects;                                    // 目标检测结果
    std::vector<bool>    mvbKeysStatic;                                    // whether point is static,目标框内的特征点
    std::vector<int>     mvKeysPointsId;


    GEOM_FADE25D::Fade_2D *mpDT_ground;                                    //地面点三角网

    // 相机与激光雷达之间的外参
    Eigen::Matrix4f mLidar2CameraExtric;

    static std::ofstream mEfficiency_fs;
private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
