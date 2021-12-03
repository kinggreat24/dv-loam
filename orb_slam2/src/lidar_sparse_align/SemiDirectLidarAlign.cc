/*
 * @Author: kinggreat24
 * @Date: 2021-08-16 15:33:30
 * @LastEditTime: 2021-08-26 17:00:46
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/src/lidar_sparse_align/SemiDirectLidarAlign.cc
 * 可以输入预定的版权声明、个性签名、空行等
 */

#include "lidar_sparse_align/SemiDirectLidarAlign.h"

namespace ORB_SLAM2
{

    SemiDirectLidarAlign::SemiDirectLidarAlign(const vk::PinholeCamera *pinhole_model,
                                               const ORB_SLAM2::_tracker_t &tracker_info)
    {
        min_level_ = tracker_info_.min_level;
        max_level_ = tracker_info_.max_level;
        use_weight_scale_ = tracker_info_.use_weight_scale;
        set_weightfunction();
    }

    SemiDirectLidarAlign::~SemiDirectLidarAlign() {}

    // 具有深度信息的特征点，使用重投影误差
    bool SemiDirectLidarAlign::semi_direct_tracking(Frame *reference, Frame *current,
        std::vector<cv::DMatch> &feature_match, Sophus::SE3 &transformation)
    {
        // 激光点处的patch
        ref_image_pyramid_ = reference->mvImgPyramid;
        pointcloud_ref_ = reference->pointcloud();
        cur_image_pyramid_ = current->mvImgPyramid;

        // 间接法相关
        ref_cur_feature_match_ = feature_match;
        ref_keypoints_ = reference->mvKeysUn;
        cur_keypoints_ = current_->mvKeysUn;
        ref_keypoints_depth_ = reference->mvDepth;

        // keypoints_point3d_ref_.clear;
        ref_keypoints_3d_.resize(reference->N, Eigen::Vector3d(-1, -1, -1));
        inlier_.resize(reference->N, true);
        for (size_t i = 0; i < ref_keypoints_depth_.size(); i++)
        {
            float depth = ref_keypoints_depth_[i];
            if (depth <= 0)
                continue;

            cv::Point2f kp = ref_keypoints_.at(i).pt;

            Eigen::Vector3d pt_3d_vec = depth * pinhole_model_->cam2world(Eigen::Vector2d(kp.x, kp.y));

            pcl::PointXYZI pt_3d;
            pt_3d.x = pt_3d_vec.x();
            pt_3d.y = pt_3d_vec.y();
            pt_3d.z = pt_3d_vec.z();
            pt_3d.intensity = i + 10000;

            pointcloud_ref_.push_back(pt_3d);
            // keypoints_point3d_ref_.push_back(pt_3d);

            ref_keypoints_3d_[i] = Eigen::Vector3d(pt_3d.x, pt_3d.y, pt_3d.z);
        }
    }

    void SemiDirectLidarAlign::set_weightfunction()
    {
        // 直接法权重函数，t-distribution
        if (tracker_info_.use_weight_scale)
        {
            switch (tracker_info_.scale_estimator_type)
            {
            case ScaleEstimatorType::TDistributionScale:
                scale_estimator_.reset(new TDistributionScaleEstimator());
                break;
            default:
                cerr << "Do not use scale estimator." << endl;
            }
        }

        switch (tracker_info_.weight_function_type)
        {
        case WeightFunctionType::TDistributionWeight:
            weight_function_.reset(new TDistributionWeightFunction());
            break;
        default:
            cerr << "Do not use weight function." << endl;
        }

        // 间接法权重函数，gaussian-distribution, 使用huber-kerner
    }

    // 计算参考帧图像块
    void SemiDirectLidarAlign::precompute_patches(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI> &pointcloud,
                                                  cv::Mat &patch_buf)
    {
        const int border = patch_halfsize_ + 2 + 2;
        const int stride = img.cols;
        const float scale = 1.0f / (1 << current_level_);

        std::vector<Eigen::Vector2d> uv_set;
        for (auto pt = pointcloud.begin(); pt != pointcloud.end(); pt++)
        {
            Eigen::Vector3d xyz(pt->x, pt->y, pt->z);
            Eigen::Vector2d uv = scale * pinhole_model_->world2cam(xyz);
            uv_set.push_back(uv);
        }

        patch_buf = cv::Mat(pointcloud.size(), pattern_length_, CV_32F);

        auto pc_iter = pointcloud.begin();
        size_t point_counter = 0;

        for (auto uv_iter = uv_set.begin(); uv_iter != uv_set.end(); ++uv_iter, ++pc_iter, ++point_counter)
        {
            Eigen::Vector2d &uv = *uv_iter;
            float u_f = uv(0);
            float v_f = uv(1);
            const int u_i = static_cast<int>(u_f);
            const int v_i = static_cast<int>(v_f);

            if (u_i - border < 0 || u_i + border > img.cols || v_i - border < 0 || v_i + border > img.rows || pc_iter->z <= 0.0)
            {
                float *patch_buf_ptr = reinterpret_cast<float *>(patch_buf.data) + pattern_length_ * point_counter;
                for (int i = 0; i < pattern_length_; ++i, ++patch_buf_ptr)
                    *patch_buf_ptr = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            const float subpix_u = u_f - u_i;
            const float subpix_v = v_f - v_i;
            const float w_tl = (1.0 - subpix_u) * (1.0 - subpix_v);
            const float w_tr = subpix_u * (1.0 - subpix_v);
            const float w_bl = (1.0 - subpix_u) * subpix_v;
            const float w_br = subpix_u * subpix_v;

            size_t pixel_counter = 0;

            float *patch_buf_ptr = reinterpret_cast<float *>(patch_buf.data) + pattern_length_ * point_counter;

            for (int i = 0; i < pattern_length_; ++i, ++pixel_counter, ++patch_buf_ptr)
            {
                int x = pattern_[i][0];
                int y = pattern_[i][1];

                float *img_ptr = (float *)img.data + (v_i + y) * stride + (u_i + x);
                *patch_buf_ptr = w_tl * img_ptr[0] + w_tr * img_ptr[1] + w_bl * img_ptr[stride] + w_br * img_ptr[stride + 1];
            }
        }
    }

    /**
 * 计算当前帧激光点patch块的像素值，梯度信息以及雅克比矩阵
 * @param img               当前帧图像
 * @param pointcloud_c      激光点
 * @param patch_buf         激光点对应的图像块
 * @param dI_buf output parameter of covariance of match
 * @param jacobian_buf whether to penalize matches further from the search center
 * @return strength of response
 */
    void SemiDirectLidarAlign::precompute_patches(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI> &pointcloud_c,
                                                  cv::Mat &patch_buf,
                                                  Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> &dI_buf,
                                                  Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> &jacobian_buf)
    {
        const int border = patch_halfsize_ + 2 + 2;
        const int stride = img.cols;
        const float scale = 1.0f / (1 << current_level_);

        std::vector<Eigen::Vector2d> uv_set;
        for (auto pt = pointcloud_c.begin(); pt != pointcloud_c.end(); pt++)
        {
            Eigen::Vector3d xyz(pt->x, pt->y, pt->z);
            Eigen::Vector2d uv = scale * pinhole_model_->world2cam(xyz);
            uv_set.push_back(uv);
        }

        patch_buf = cv::Mat(pointcloud_c.size(), pattern_length_, CV_32F);

        auto pc_iter = pointcloud_c.begin();
        size_t point_counter = 0;

        // 计算每个patch的像素值以及对应的雅克比矩阵
        for (auto uv_iter = uv_set.begin(); uv_iter != uv_set.end(); ++uv_iter, ++pc_iter, ++point_counter)
        {
            Eigen::Vector2d &uv = *uv_iter;
            float u_f = uv(0);
            float v_f = uv(1);
            const int u_i = static_cast<int>(u_f);
            const int v_i = static_cast<int>(v_f);

            if (u_i - border < 0 || u_i + border > img.cols || v_i - border < 0 || v_i + border > img.rows || pc_iter->z <= 0.0)
            {
                float *patch_buf_ptr = reinterpret_cast<float *>(patch_buf.data) + pattern_length_ * point_counter;
                for (int i = 0; i < pattern_length_; ++i, ++patch_buf_ptr)
                    *patch_buf_ptr = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            const float subpix_u = u_f - u_i;
            const float subpix_v = v_f - v_i;
            const float w_tl = (1.0 - subpix_u) * (1.0 - subpix_v);
            const float w_tr = subpix_u * (1.0 - subpix_v);
            const float w_bl = (1.0 - subpix_u) * subpix_v;
            const float w_br = subpix_u * subpix_v;

            size_t pixel_counter = 0;

            float *patch_buf_ptr = reinterpret_cast<float *>(patch_buf.data) + pattern_length_ * point_counter;

            for (int i = 0; i < pattern_length_; ++i, ++pixel_counter, ++patch_buf_ptr)
            {
                int x = pattern_[i][0];
                int y = pattern_[i][1];

                float *img_ptr = (float *)img.data + (v_i + y) * stride + (u_i + x);
                *patch_buf_ptr = w_tl * img_ptr[0] + w_tr * img_ptr[1] + w_bl * img_ptr[stride] + w_br * img_ptr[stride + 1];

                // 计算雅克比矩阵
                // precompute image gradient
                float dx = 0.5f * ((w_tl * img_ptr[1] + w_tr * img_ptr[2] + w_bl * img_ptr[stride + 1] + w_br * img_ptr[stride + 2]) - (w_tl * img_ptr[-1] + w_tr * img_ptr[0] + w_bl * img_ptr[stride - 1] + w_br * img_ptr[stride]));
                float dy = 0.5f * ((w_tl * img_ptr[stride] + w_tr * img_ptr[1 + stride] + w_bl * img_ptr[stride * 2] + w_br * img_ptr[stride * 2 + 1]) - (w_tl * img_ptr[-stride] + w_tr * img_ptr[1 - stride] + w_bl * img_ptr[0] + w_br * img_ptr[1]));

                Matrix2x6 frame_jac;
                Eigen::Vector3f xyz(pc_iter->x, pc_iter->y, pc_iter->z);
                Frame::jacobian_xyz2uv(xyz, frame_jac);

                Eigen::Vector2f dI_xy(dx, dy);
                dI_buf.col(point_counter * pattern_length_ + i) = dI_xy;
                jacobian_buf.col(point_counter * pattern_length_ + pixel_counter) = (dx * pinhole_model_->fx() * frame_jac.row(0) + dy * pinhole_model_->fy() * frame_jac.row(1)) / (1 << current_level_);
            }
        }
    }

    // 计算重投影误差以及雅克比矩阵
    void SemiDirectLidarAlign::precompute_reproject_error(const Sophus::SE3 &T_cur_ref)
    {
        Eigen::Matrix3d R_cur_ref = T_cur_ref.rotation_matrix();
        Eigen::Vector3d t_cur_ref = T_cur_ref.translation();

        pre_weight_res_p_.resize(inlier_.size(), -1.0);

        for (size_t i = 0; i < ref_cur_feature_match_.size(); i++)
        {
            cv::DMatch dm = ref_cur_feature_match_.at(i);
            if (dm.queryIdx < 0)
                continue;

            float depth = ref_keypoints_depth_.at(dm.queryIdx);
            if (depth < 0)
                continue;

            // 外点
            if (!inlier_.at(dm.queryIdx))
                continue;

            // 重投影误差
            Eigen::Vector3d pt_ref = ref_keypoints_3d_.at(dm.queryIdx);
            Eigen::Vector3d xyz_trans = R_cur_ref * pt_ref + t_cur_ref;

            Eigen::Vector2d kp_reproj = pinhole_model_->world2cam(xyz_trans);

            Eigen::Vector2f p_err;
            p_err[0] = ref_keypoints_.at(dm.trainIdx).pt.x - kp_reproj.x();
            p_err[1] = ref_keypoints_.at(dm.trainIdx).pt.y - kp_reproj.y();

            reproj_err_buf_.col(i * 2) = p_err;
            pre_weight_res_p_[dm.queryIdx] = p_err.norm();

            // 重投影误差雅克比矩阵
            double x = xyz_trans[0];
            double y = xyz_trans[1];
            double z = xyz_trans[2];
            double z_2 = z * z;
            double inv_z = 1.0 / z;
            double inv_z2 = 1.0 / z_2;

            double fx_ = pinhole_model_->fx();
            double fy_ = pinhole_model_->fy();
            double cx_ = pinhole_model_->cx();
            double cy_ = pinhole_model_->cy();

            Eigen::Matrix<float, 2, 6> _jacobian = Eigen::Matrix<float, 2, 6>::Zero();
            _jacobian(0, 0) =  fx_ * x * y * inv_z2;
            _jacobian(0, 1) =  fx_ * x * inv_z2 + fx_;
            _jacobian(0, 2) = -1.0 * fx_ * y * inv_z;
            _jacobian(0, 3) =  fx_ * inv_z;
            _jacobian(0, 4) =  0;
            _jacobian(0, 5) = -1.0 * fx_ * x * inv_z2;

            _jacobian(1, 0) = -1.0 * (fy_ + fy_ * y * y * inv_z2);
            _jacobian(1, 1) =  fy_ * x * y * inv_z2;
            _jacobian(1, 2) =  fy_ * x * inv_z;
            _jacobian(1, 3) =  0.0;
            _jacobian(1, 4) =  fy_ * inv_z;
            _jacobian(1, 5) = -1.0 * fy_ * y * inv_z2;

            _jacobian = -1.0 * _jacobian;

            reproj_jacobian_buf_.col(i * 2) = _jacobian.row(0);
            reproj_jacobian_buf_.col(i * 2 + 1) = _jacobian.row(1);
        }
    }

    double SemiDirectLidarAlign::compute_residuals(const Sophus::SE3 &T_cur_ref)
    {
        errors_.clear();
        J_.clear();
        weight_.clear();

        if (!is_precomputed_)
        {
            // 每一层金字塔的第一次迭代，需要重新计算参考帧图像块的信息
            if (!ref_patch_buf_.empty())
                ref_patch_buf_.release();

            cv::Mat reference_img = ref_image_pyramid_[current_level_].clone();
            Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> dI_buf_;
            precompute_patches(reference_img, pointcloud_ref_, ref_patch_buf_);

            // std::cout<<"Init dI_buf & jacobian_buf"<<std::endl;
            dI_buf_.resize(Eigen::NoChange, ref_patch_buf_.rows * pattern_length_);
            dI_buf_.setZero();
            jacobian_buf_.resize(Eigen::NoChange, ref_patch_buf_.rows * pattern_length_);
            jacobian_buf_.setZero();

            // 间接法误差与雅克比矩阵
            int valid_feature_measuremets = 0;
            for (size_t i = 0; i < ref_cur_feature_match_.size(); i++)
            {
                cv::DMatch dm = ref_cur_feature_match_.at(i);
                if (dm.queryIdx < 0)
                    continue;

                float depth = ref_keypoints_depth_.at(i);
                if (depth <= 0)
                    continue;

                // 是否是内点
                if (!inlier_.at(i))
                    continue;

                valid_feature_measuremets++;
            }
            reproj_err_buf_.resize(Eigen::NoChange, valid_feature_measuremets);
            reproj_err_buf_.setZero();
            reproj_jacobian_buf_.resize(Eigen::NoChange, valid_feature_measuremets);
            reproj_jacobian_buf_.setZero();

            is_precomputed_ = true;
        }

        // TODO:把这一部分封装成一个函数，方便实现多线程计算
        // 计算激光点对应的图像块以及雅克比矩阵
        cv::Mat current_img = cur_image_pyramid_[current_level_].clone();
        pcl::PointCloud<pcl::PointXYZI> pointcloud_cur;
        pcl::transformPointCloud(pointcloud_ref_, pointcloud_cur, T_cur_ref.matrix());
        cv::Mat cur_lidar_points_patch_buf;
        precompute_patches(current_img, pointcloud_cur, cur_lidar_points_patch_buf, dI_buf_, jacobian_buf_);

        // 计算直接法权重
        cv::Mat errors = cv::Mat(pointcloud_cur.size(), pattern_length_, CV_32F);
        errors = cur_patch_buf_ - ref_patch_buf_;
        scale_ = scale_estimator_->compute(errors);

        float chi2 = 0.0f;
        n_measurement_ = 0;

        float *errors_ptr = errors.ptr<float>();
        float *ref_patch_buf_ptr = ref_patch_buf_.ptr<float>();
        float *cur_patch_buf_ptr = cur_patch_buf_.ptr<float>();

        float IiIj = 0.0f;
        float IiIi = 0.0f;
        float sum_Ii = 0.0f;
        float sum_Ij = 0.0f;

        for (int i = 0; i < errors.size().area(); ++i, ++errors_ptr, ++ref_patch_buf_ptr, ++cur_patch_buf_ptr)
        {

            float &res = *errors_ptr;

            float &Ii = *ref_patch_buf_ptr;
            float &Ij = *cur_patch_buf_ptr;

            if (std::isfinite(res))
            {
                n_measurement_++;

                Vector6 J(jacobian_buf_.col(i));

                errors_.push_back(res);
                J_.push_back(J);

                IiIj += Ii * Ij;
                IiIi += Ii * Ii;
                sum_Ii += Ii;
                sum_Ij += Ij;
            }
        }

        affine_a_ = IiIj / IiIi;
        affine_b_ = (sum_Ij - affine_a_ * sum_Ii) / n_measurement_;

        vector<float> sorted_errors;
        sorted_errors.resize(errors_.size());
        copy(errors_.begin(), errors_.end(), sorted_errors.begin());
        sort(sorted_errors.begin(), sorted_errors.end());

        float median_mu = sorted_errors[sorted_errors.size() / 2];

        std::vector<float> absolute_res_error;
        for (auto error : errors_)
        {
            absolute_res_error.push_back(fabs(error - median_mu));
        }
        sort(absolute_res_error.begin(), absolute_res_error.end());
        float median_abs_deviation = 1.4826 * absolute_res_error[absolute_res_error.size() / 2];

        for (auto error : errors_)
        {
            float weight = 1.0;
            weight = weight_function_->weight((error - median_mu) / median_abs_deviation);
            weight_.push_back(weight);

            chi2 += error * error * weight;
        }

        // TODO:把这一部分封装成一个函数，方便实现多线程计算

        // point features pre-weight computation
        precompute_reproject_error(T_cur_ref);

        // estimate scale of the residuals
        double s_p = 1.0;
        double th_min = 0.0001;
        double th_max = sqrt(7.815);
        s_p = vector_stdv_mad(pre_weight_res_p_);
        if (s_p < th_min)
            s_p = th_min;
        if (s_p > th_max)
            s_p = th_max;

        // if employing robust cost function
        for (size_t i = 0; i < pre_weight_res_p_.size(); i++)
        {
            double r = pre_weight_res_p_.at(i);

            // 剔除没有匹配成功的，或者没有深度的，或者是外点的匹配对
            if (r < 0)
                continue;

            double w = 1.0;
            double x = r / s_p;
            // w = robustWeightCauchy(x) ;
        }
    }

    void SemiDirectLidarAlign::max_level(int level) { max_level_ = level; }

    double SemiDirectLidarAlign::build_LinearSystem(Sophus::SE3 &model)
    {
        double res = compute_residuals(model);

        //是否显示残差图像
        // if(display_)
        //     resimg_ = cv::Mat(current_->level(current_level_).size(), CV_32F, cv::Scalar(0));

        H_.setZero();
        Jres_.setZero();

        for (int i = 0; i < errors_.size(); ++i)
        {
            float &res = errors_[i];
            Vector6 &J = J_[i];
            float &weight = weight_[i];

            H_.noalias() += J * J.transpose() * weight;
            Jres_.noalias() -= J * res * weight;

            // if(display_)
            //     resimg_.at<float>((int) v_cur + y - patch_halfsize_, (int) u_cur + x - patch_halfsize_) =
            //             res / 255.0;
        }

        // std::cout<<"Hessian matrix: "<<std::endl<<H_.matrix()<<std::endl;

        return res;
    }

    // implementation for LSQNonlinear class
    void SemiDirectLidarAlign::update(const ModelType &old_model, ModelType &new_model)
    {
        Eigen::Matrix<double, 6, 1> update_;
        for (int i = 0; i < 6; i++)
            update_[i] = -x_[i];
        new_model = Sophus::SE3::exp(update_) * old_model;
    }

    void SemiDirectLidarAlign::removeOutliers(Eigen::Matrix4d DT)
    {

        //TODO: if not usig mad stdv, use just a fixed threshold (sqrt(7.815)) to filter outliers (with a single for loop...)

        // // point features
        // vector<double> res_p;
        // res_p.reserve(matched_pt.size());
        // int iter = 0;
        // for( auto it = matched_pt.begin(); it!=matched_pt.end(); it++, iter++)
        // {
        //     // projection error
        //     Vector3d P_ = DT.block(0,0,3,3) * (*it)->P + DT.col(3).head(3);
        //     Vector2d pl_proj = cam->projection( P_ );
        //     res_p.push_back( ( pl_proj - (*it)->pl_obs ).norm() * sqrt((*it)->sigma2) );
        //     //res_p.push_back( ( pl_proj - (*it)->pl_obs ).norm() );
        // }
        // // estimate robust parameters
        // double p_stdv, p_mean, inlier_th_p;
        // vector_mean_stdv_mad( res_p, p_mean, p_stdv );
        // inlier_th_p = Config::inlierK() * p_stdv;
        // //inlier_th_p = sqrt(7.815);
        // //cout << endl << p_mean << " " << p_stdv << "\t" << inlier_th_p << endl;
        // // filter outliers
        // iter = 0;
        // for( auto it = matched_pt.begin(); it!=matched_pt.end(); it++, iter++)
        // {
        //     if( (*it)->inlier && fabs(res_p[iter]-p_mean) > inlier_th_p )
        //     {
        //         (*it)->inlier = false;
        //         n_inliers--;
        //         n_inliers_pt--;
        //     }
        // }
    }

}