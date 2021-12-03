/*
 * @Author: Kinggreat24
 * @Date: 2020-05-19 22:25:01
 * @LastEditors: kinggreat24
 * @LastEditTime: 2021-08-16 19:40:13
 * @Description: 
 */ 
#ifndef SLIDE_WINDOW_SPARSE_LIDAR_ALIGN_H
#define SLIDE_WINDOW_SPARSE_LIDAR_ALIGN_H

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

class SlideWindowSparseAlign  : public LSQNonlinearGaussNewton <6, Sophus::SE3>  //LSQNonlinearGaussNewton <6, Sophus::SE3f> LSQNonlinearLevenbergMarquardt <6, Sophus::SE3f>
{
    static const int patch_halfsize_ = 2;
    static const int patch_size_ = 2*patch_halfsize_;
    static const int patch_area_ = patch_size_*patch_size_;

    static const int pattern_length_ = 8;
    int pattern_[8][2] = { {0, 0}, {2, 0}, {1, 1}, {0, -2}, {-1, -1}, {-2, 0}, {-1, 1}, {0, 2} };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   

    SlideWindowSparseAlign(const vk::PinholeCamera* pinhole_model,const ORB_SLAM2::_tracker_t& tracker_info);
    ~SlideWindowSparseAlign();

    bool slidewindow_optimization(const std::vector<Frame>vRefFrames, Frame *current, Sophus::SE3 &Tcw);

    virtual void startIteration();
    virtual void finishIteration();
    void compute_residual_image(
        const Sophus::SE3& transformation, 
        const pcl::PointCloud<pcl::PointXYZI>::Ptr mpLidarPointCloud, 
        const int level = 0);

    void compute_residual_image(
        const cv::Mat& ref_img, const cv::Mat& cur_img,
        cv::Mat& residual_img,const Sophus::SE3& Tcr, 
        const pcl::PointCloud<pcl::PointXYZI>::Ptr ref_pointclouds);

private:
    int current_level_;

    int min_level_;
    int max_level_;

    bool display_;                  //!< display residual image.
    cv::Mat resimg_;                // residual image.
 
    const vk::PinholeCamera* pinhole_model_;

    Sophus::SE3 Tji_;

    Frame* reference_;                                  // 上一帧图像
    Frame* current_;                                    
    // 当前帧图像

    pcl::PointCloud<pcl::PointXYZI> pointcloud_ref_;    //参考帧激光点云
    std::vector<cv::Mat> ref_image_pyramid_;            //参考帧图像金字塔
    std::vector<cv::Mat> cur_image_pyramid_;            //当前帧图像金字塔

    std::vector<Frame> mv_ref_frames_;                  //局部窗口图像

    bool is_precomputed_;

    // 激光点直接法对应的雅克比矩阵
    cv::Mat ref_patch_buf_, cur_patch_buf_;
    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf_;
    Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_buf_;

    // 特征点直接法
    Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> kp_dI_buf_;
    Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> kp_jacobian_buf_;

    // 重投影误差对应的雅克比矩阵 
    Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> reprojection_jacobian_buf_;


    vector<float> errors_;
    vector<Vector6> J_;
    vector<float> weight_;

    float affine_a_;
    float affine_b_;

    // void precompute_patches(cv::Mat& img, pcl::PointCloud<pcl::PointXYZI>& pointcloud, cv::Mat& patch_buf,
    //     Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf, 
    //     Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> jacobian_buf, bool is_derivative);
        
    void precompute_patches(cv::Mat &img, 
        pcl::PointCloud<pcl::PointXYZI> &pointcloud_c, 
        cv::Mat &patch_buf, 
        Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> &dI_buf, 
        Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> &jacobian_buf, int index);

    void precompute_patches(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI> &pointcloud, 
        cv::Mat &patch_buf);

    double compute_residuals(const Sophus::SE3& transformation);
   
    // implementation for LSQNonlinear class
    virtual void update (const ModelType &old_model, ModelType &new_model);


public:
    // weight function
    ORB_SLAM2::_tracker_t tracker_info_;
    bool use_weight_scale_;
    float scale_;
    std::shared_ptr<ScaleEstimator> scale_estimator_;
    std::shared_ptr<WeightFunction> weight_function_;
    void set_weightfunction();
    void max_level(int level);

protected:
    virtual double build_LinearSystem(Sophus::SE3& model);
};

}// end of namespace dedvo

#endif//