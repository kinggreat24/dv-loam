/*
 * @Author: kinggreat24
 * @Date: 2021-08-16 15:33:44
 * @LastEditTime: 2021-08-18 16:00:04
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/include/lidar_sparse_align/SemiDirectLidarAlign.h
 * 可以输入预定的版权声明、个性签名、空行等
 */

#ifndef SEMI_DIRECT_LIDAR_ALIGN_H
#define SEMI_DIRECT_LIDAR_ALIGN_H

#include <iostream>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <sophus/se3.h>

#include "lidar_sparse_align/WeightFunction.h"
#include "lidar_sparse_align/LSQNonlinear.hpp"

#include "Frame.h"
#include "KeyFrame.h"

#include <vikit/pinhole_camera.h>
#include <vikit/vision.h>

namespace ORB_SLAM2{

// 利用匹配的图像点和直接法联合进行里程计的计算
class SemiDirectLidarAlign : public LSQNonlinearGaussNewton <6, Sophus::SE3> {
    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2*patch_halfsize_;
    static const int patch_area_ = patch_size_*patch_size_;

    static const int pattern_length_ = 8;
    int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };

public:
    SemiDirectLidarAlign(const vk::PinholeCamera* pinhole_model,const ORB_SLAM2::_tracker_t& tracker_info);
    ~SemiDirectLidarAlign();
    
    bool semi_direct_tracking(Frame *reference, Frame *current, std::vector<cv::DMatch>& feature_match, Sophus::SE3 &transformation);

private:
    int current_level_;

    int min_level_;
    int max_level_;

    bool display_;                  //!< display residual image.
    cv::Mat resimg_;                // residual image.
 
    const vk::PinholeCamera* pinhole_model_;

    Sophus::SE3 Tji_;

    // 
    Frame* reference_;                                  // 上一帧图像
    Frame* current_;                                    
    // 当前帧图像

    pcl::PointCloud<pcl::PointXYZI> pointcloud_ref_;    // 参考帧激光点云(采样出来梯度比较大的点)
    std::vector<cv::Mat> ref_image_pyramid_;            // 参考帧图像金字塔
    std::vector<cv::Mat> cur_image_pyramid_;            // 当前帧图像金字塔

    std::vector<float> ref_keypoints_depth_;                 // 参考帧特征点的深度(可以通过激光进行深度拟合)
    std::vector<bool>  inlier_;                              // 匹配是否为内点
    std::vector<cv::KeyPoint> ref_keypoints_;                // 参考帧特征点
    std::vector<cv::KeyPoint> cur_keypoints_;                // 当前帧特征点
    std::vector<cv::DMatch> ref_cur_feature_match_;          // 当前帧与参考帧的特征匹配结果
    std::vector<Eigen::Vector3d> ref_keypoints_3d_;

    bool is_precomputed_;                                    // 是否已经计算过梯度
    cv::Mat ref_patch_buf_, cur_patch_buf_;
    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf_;
    Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_buf_;

    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> reproj_err_buf_;
    Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> reproj_jacobian_buf_;

    // 直接法相关
    vector<float> errors_;
    vector<Vector6> J_;
    vector<float> weight_;
    float affine_a_;
    float affine_b_;

    // 间接法对应的加权后误差、雅克比矩阵以及权重
    std::vector<double> pre_weight_res_p_;
    vector<float> reproj_errors_;
    vector<Vector6> reproj_J_;
    vector<float> reproj_weight_;

    void precompute_patches(cv::Mat& img, pcl::PointCloud<pcl::PointXYZI>& pointcloud, 
        cv::Mat& patch_buf);
    
    // 直接法梯度计算
    void precompute_patches(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI> &pointcloud_c, 
        cv::Mat &patch_buf, 
        Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> &dI_buf, 
        Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> &jacobian_buf);

    void precompute_reproject_error(const Sophus::SE3& T_cur_ref);


    double compute_residuals(const Sophus::SE3& transformation);
   
    // implementation for LSQNonlinear class
    virtual void update (const ModelType &old_model, ModelType &new_model);

    void removeOutliers(Eigen::Matrix4d DT);
public:
    // weight function
    ORB_SLAM2::_tracker_t tracker_info_;
    bool use_weight_scale_;
    float scale_;

    // 直接法相关的权重函数
    std::shared_ptr<ScaleEstimator> scale_estimator_;
    std::shared_ptr<WeightFunction> weight_function_;

    // 间接法相关的权重函数
    std::shared_ptr<ScaleEstimator> feature_scale_estimator_;
    std::shared_ptr<WeightFunction> feature_weight_function_;


    void set_weightfunction();
    void max_level(int level);

protected:
    virtual double build_LinearSystem(Sophus::SE3& model);
};

}



#endif//SEMI_DIRECT_LIDAR_ALIGN_H