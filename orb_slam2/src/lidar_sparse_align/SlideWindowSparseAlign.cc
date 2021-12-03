/*
 * @Author: Kinggreat24
 * @Date: 2020-08-13 11:19:31
 * @LastEditors: kinggreat24
 * @LastEditTime: 2021-08-17 14:45:49
 * @Description: 
 */
#include "lidar_sparse_align/SlideWindowSparseAlign.h"

namespace ORB_SLAM2
{

    SlideWindowSparseAlign::SlideWindowSparseAlign(const vk::PinholeCamera *pinhole_model, const ORB_SLAM2::_tracker_t &tracker_info)
        : pinhole_model_(pinhole_model), tracker_info_(tracker_info)
    {
        // tracker_info_ = dvlo::Config::cfg()->tracker();
        min_level_ = tracker_info_.min_level;
        max_level_ = tracker_info_.max_level;
        use_weight_scale_ = tracker_info_.use_weight_scale;
        set_weightfunction();
    }

    SlideWindowSparseAlign::~SlideWindowSparseAlign()
    {
    }

    void SlideWindowSparseAlign::set_weightfunction()
    {
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
    }

    bool SlideWindowSparseAlign::slidewindow_optimization(const std::vector<Frame>vReferenceFrames, 
        Frame *current, Sophus::SE3 &Tcw)
    {
        bool status = true;

        //局部窗口图像
        mv_ref_frames_     = vReferenceFrames;
        cur_image_pyramid_ = current->mvImgPyramid;
        
        affine_a_ = 1.0f;
        affine_b_ = 0.0f;

        for (current_level_ = max_level_; current_level_ >= min_level_; current_level_--)
        {
            is_precomputed_ = false;
            stop_ = false;
            optimize(Tcw);
        }

        return status;
    }


    void SlideWindowSparseAlign::startIteration() {}
    void SlideWindowSparseAlign::finishIteration() {}

    void SlideWindowSparseAlign::compute_residual_image(
        const Sophus::SE3 &Tcr, const pcl::PointCloud<pcl::PointXYZI>::Ptr mpLidarPointCloud,
        const int level)
    {
        cv::Mat reference_img = reference_->level(level).clone();
        cv::Mat current_img = current_->level(level).clone();

        cv::Mat residual_img = cv::Mat(reference_img.rows, reference_img.cols, CV_32F, cv::Scalar(0));
        cv::Mat residual_mask_img = reference_img.clone();
        if (residual_mask_img.channels() == 1)
            cv::cvtColor(residual_mask_img, residual_mask_img, CV_GRAY2BGR);

        const int border = patch_halfsize_ + 2 + 2;
        const int stride = reference_img.cols;
        const float scale = 1.0f / (1 << level);

        Eigen::Matrix<float, 8, 1> ref_photometric_error;
        Eigen::Matrix<float, 8, 1> cur_photometric_error;

        float v_min = 0.15;
        float v_max = 0.5;
        float dv = v_max - v_min;
        for (size_t i = 0; i < mpLidarPointCloud->size(); i++)
        {
            //计算参考帧的光度
            pcl::PointXYZI pt_ref = mpLidarPointCloud->points[i];
            if (pt_ref.z <= 0)
                continue;
            Eigen::Vector3d xyz(pt_ref.x, pt_ref.y, pt_ref.z);
            Eigen::Vector2d uv_ref = scale * pinhole_model_->world2cam(xyz);

            {
                const float u_f = uv_ref[0];
                const float v_f = uv_ref[1];
                const int u_i = static_cast<int>(u_f);
                const int v_i = static_cast<int>(v_f);

                if (!pinhole_model_->isInFrame(uv_ref.cast<int>(), border, level))
                    continue;

                //差值
                const float subpix_u = u_f - u_i;
                const float subpix_v = v_f - v_i;
                const float w_tl = (1.0 - subpix_u) * (1.0 - subpix_v);
                const float w_tr = subpix_u * (1.0 - subpix_v);
                const float w_bl = (1.0 - subpix_u) * subpix_v;
                const float w_br = subpix_u * subpix_v;

                for (int j = 0; j < pattern_length_; ++j)
                {
                    int x = pattern_[j][0];
                    int y = pattern_[j][1];

                    ref_photometric_error[j] = vk::interpolateMat_32f(reference_img, u_i + x, v_i + y);
                }
            }

            //计算当前帧的光度
            Eigen::Vector3d pt_cur = Tcr * xyz;
            if (pt_cur.z() <= 0)
                continue;

            Eigen::Vector2d uv_cur = scale * pinhole_model_->world2cam(pt_cur);
            {
                const float u_f = uv_cur[0];
                const float v_f = uv_cur[1];
                const int u_i = static_cast<int>(u_f);
                const int v_i = static_cast<int>(v_f);

                if (!pinhole_model_->isInFrame(uv_cur.cast<int>(), border, level))
                    continue;

                const float subpix_u = u_f - u_i;
                const float subpix_v = v_f - v_i;
                const float w_tl = (1.0 - subpix_u) * (1.0 - subpix_v);
                const float w_tr = subpix_u * (1.0 - subpix_v);
                const float w_bl = (1.0 - subpix_u) * subpix_v;
                const float w_br = subpix_u * subpix_v;

                for (int j = 0; j < pattern_length_; ++j)
                {
                    int x = pattern_[j][0];
                    int y = pattern_[j][1];

                    cur_photometric_error[j] = vk::interpolateMat_32f(current_img, u_i + x, v_i + y);
                }
            }

            //计算灰度误差
            float err = (cur_photometric_error - (affine_a_ * ref_photometric_error)).norm() + affine_b_;
            float v = err;
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

            if (err < v_min)
                continue;

            residual_img.at<float>(uv_ref[1], uv_ref[0]) = err;
            cv::circle(residual_mask_img, cv::Point2f(uv_ref[0], uv_ref[1]), 1, 255 * cv::Scalar(r, g, b), -1);
        }

        // double sum = std::accumulate(std::begin(err_vecs), std::end(err_vecs), 0.0);
        // double mean =  sum / err_vecs.size(); //均值
        // std::cout<<"mean: "<<mean<<std::endl;

        char file_name[128] = {0};
        sprintf(file_name, "/home/kinggreat24/pc/residual_image_%d.png", reference_->mnId);
        cv::imwrite(file_name, 255 * residual_mask_img);
    }

    void SlideWindowSparseAlign::compute_residual_image(
        const cv::Mat &reference_img, const cv::Mat &current_img,
        cv::Mat &residual_img, const Sophus::SE3 &Tcr,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr ref_pointclouds)
    {
        // cv::Mat residual_img = cv::Mat(reference_img.rows,reference_img.cols,CV_32F,cv::Scalar(0));
        residual_img = cv::Mat(reference_img.rows, reference_img.cols, CV_32F, cv::Scalar(0));
        cv::Mat residual_mask_img = reference_img.clone();
        if (residual_mask_img.channels() == 1)
            cv::cvtColor(residual_mask_img, residual_mask_img, CV_GRAY2BGR);

        const int level = 0;
        const int border = patch_halfsize_ + 2 + 2;
        const int stride = reference_img.cols;
        const float scale = 1.0f / (1 << level);

        Eigen::Matrix<float, 8, 1> ref_photometric_error;
        Eigen::Matrix<float, 8, 1> cur_photometric_error;

        float v_min = 0.15;
        float v_max = 0.5;
        float dv = v_max - v_min;
        for (size_t i = 0; i < ref_pointclouds->size(); i++)
        {
            //计算参考帧的光度
            pcl::PointXYZI pt_ref = ref_pointclouds->points[i];
            if (pt_ref.z <= 0)
                continue;
            Eigen::Vector3d xyz(pt_ref.x, pt_ref.y, pt_ref.z);
            Eigen::Vector2d uv_ref = scale * pinhole_model_->world2cam(xyz);

            {
                const float u_f = uv_ref[0];
                const float v_f = uv_ref[1];
                const int u_i = static_cast<int>(u_f);
                const int v_i = static_cast<int>(v_f);

                if (!pinhole_model_->isInFrame(uv_ref.cast<int>(), border, level))
                    continue;

                //差值
                const float subpix_u = u_f - u_i;
                const float subpix_v = v_f - v_i;
                const float w_tl = (1.0 - subpix_u) * (1.0 - subpix_v);
                const float w_tr = subpix_u * (1.0 - subpix_v);
                const float w_bl = (1.0 - subpix_u) * subpix_v;
                const float w_br = subpix_u * subpix_v;

                for (int j = 0; j < pattern_length_; ++j)
                {
                    int x = pattern_[j][0];
                    int y = pattern_[j][1];

                    ref_photometric_error[j] = vk::interpolateMat_32f(reference_img, u_i + x, v_i + y);
                }
            }

            //计算当前帧的光度
            Eigen::Vector3d pt_cur = Tcr * xyz;
            if (pt_cur.z() <= 0)
                continue;

            Eigen::Vector2d uv_cur = scale * pinhole_model_->world2cam(pt_cur);
            {
                const float u_f = uv_cur[0];
                const float v_f = uv_cur[1];
                const int u_i = static_cast<int>(u_f);
                const int v_i = static_cast<int>(v_f);

                if (!pinhole_model_->isInFrame(uv_cur.cast<int>(), border, level))
                    continue;

                const float subpix_u = u_f - u_i;
                const float subpix_v = v_f - v_i;
                const float w_tl = (1.0 - subpix_u) * (1.0 - subpix_v);
                const float w_tr = subpix_u * (1.0 - subpix_v);
                const float w_bl = (1.0 - subpix_u) * subpix_v;
                const float w_br = subpix_u * subpix_v;

                for (int j = 0; j < pattern_length_; ++j)
                {
                    int x = pattern_[j][0];
                    int y = pattern_[j][1];

                    cur_photometric_error[j] = vk::interpolateMat_32f(current_img, u_i + x, v_i + y);
                }
            }

            //计算灰度误差
            float err = (cur_photometric_error - (affine_a_ * ref_photometric_error)).norm() + affine_b_;
            float v = err;
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

            if (err < v_min)
                continue;

            residual_img.at<float>(uv_ref[1], uv_ref[0]) = err;
            cv::circle(residual_mask_img, cv::Point2f(uv_ref[0], uv_ref[1]), 1, 255 * cv::Scalar(r, g, b), -1);
        }
    }

    

    void SlideWindowSparseAlign::precompute_patches(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI> &pointcloud, 
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

    //计算当前帧的patch
    void SlideWindowSparseAlign::precompute_patches(cv::Mat &img, pcl::PointCloud<pcl::PointXYZI> &pointcloud_c, 
        cv::Mat &patch_buf, 
        Eigen::Matrix<float, 2, Eigen::Dynamic, Eigen::ColMajor> &dI_buf, 
        Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> &jacobian_buf, int index)
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

        auto pc_iter         = pointcloud_c.begin();
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
                dI_buf.col(index * pattern_length_ + point_counter * pattern_length_ + i)  = dI_xy;
                jacobian_buf.col(index * pattern_length_ + point_counter * pattern_length_ + pixel_counter) = (dx * pinhole_model_->fx() * frame_jac.row(0) + dy * pinhole_model_->fy() * frame_jac.row(1)) / (1 << current_level_);
            }
        }
    }

    double SlideWindowSparseAlign::compute_residuals(const Sophus::SE3 &Tcw_cur)
    {
        errors_.clear();
        J_.clear();
        weight_.clear();

        if (!is_precomputed_)
        {
            if(!ref_patch_buf_.empty())
                ref_patch_buf_.release();

            for(size_t i=0;i<mv_ref_frames_.size();i++)
            {
                ref_image_pyramid_ = mv_ref_frames_.at(i).mvImgPyramid;
                pointcloud_ref_    = mv_ref_frames_.at(i).pointcloud();

                cv::Mat reference_path_buf;
                cv::Mat reference_img = ref_image_pyramid_[current_level_].clone();
                precompute_patches(reference_img, pointcloud_ref_, reference_path_buf);
                
                // std::cout<<"image id: "<<mv_ref_frames_.at(i).mnId<<" point size: "<<reference_path_buf.rows<<std::endl;
                
                if(ref_patch_buf_.empty())
                    ref_patch_buf_ = reference_path_buf;
                else
                    cv::vconcat(ref_patch_buf_, reference_path_buf, ref_patch_buf_); 
            }

            // std::cout<<"Init dI_buf & jacobian_buf"<<std::endl;
            dI_buf_.resize(Eigen::NoChange, ref_patch_buf_.rows * pattern_length_);
            dI_buf_.setZero();
            jacobian_buf_.resize(Eigen::NoChange, ref_patch_buf_.rows * pattern_length_);
            jacobian_buf_.setZero();

            is_precomputed_ = true;
        }


        if(!cur_patch_buf_.empty())
            cur_patch_buf_.release();
            
        int N_MEASUREMENTS = 0;
        cv::Mat current_img = cur_image_pyramid_[current_level_].clone();
        for(size_t i=0;i<mv_ref_frames_.size();i++)
        {
            Sophus::SE3 Twc_ref   = Converter::toSophusSE3( mv_ref_frames_.at(i).mTcw ).inverse();
            Sophus::SE3 T_cur_ref = Tcw_cur * Twc_ref;
         
            pcl::PointCloud<pcl::PointXYZI> pointcloud_world,pointcloud_cur;
            pointcloud_ref_       = mv_ref_frames_.at(i).pointcloud();
            pcl::transformPointCloud(pointcloud_ref_,  pointcloud_world, Twc_ref.matrix());
            pcl::transformPointCloud(pointcloud_ref_, pointcloud_cur,   T_cur_ref.matrix());
           
            cv::Mat cur_patch_buf;
            precompute_patches(current_img, pointcloud_cur, cur_patch_buf, dI_buf_, jacobian_buf_, N_MEASUREMENTS);

            if(cur_patch_buf_.empty())
                cur_patch_buf_ = cur_patch_buf;
            else
                cv::vconcat(cur_patch_buf_,cur_patch_buf,cur_patch_buf_); 

            N_MEASUREMENTS += pointcloud_world.size();
        }

        // std::cout<<"N_MEASUREMENTS: "     << N_MEASUREMENTS << std::endl;

        cv::Mat errors = cv::Mat(N_MEASUREMENTS, pattern_length_, CV_32F);
        //errors = cur_patch_buf_ - (affine_a_ * ref_patch_buf_ + affine_b_);
        errors = cur_patch_buf_ - ref_patch_buf_;

        scale_ = scale_estimator_->compute(errors);

        float chi2 = 0.0f;
        n_measurement_ = 0;

        float *errors_ptr        = errors.ptr<float>();
        float *ref_patch_buf_ptr = ref_patch_buf_.ptr<float>();
        float *cur_patch_buf_ptr = cur_patch_buf_.ptr<float>();

        float IiIj   = 0.0f;
        float IiIi   = 0.0f;
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
        return chi2 / n_measurement_;
    }

    // implementation for LSQNonlinear class
    void SlideWindowSparseAlign::update(const ModelType &old_model, ModelType &new_model)
    {
        Eigen::Matrix<double, 6, 1> update_;
        for (int i = 0; i < 6; i++)
            update_[i] = -x_[i];
        // new_model = old_model * Sophus::SE3::exp(-x_);
        new_model = Sophus::SE3::exp(update_) * old_model;
    }

    void SlideWindowSparseAlign::max_level(int level) { max_level_ = level; }



    double SlideWindowSparseAlign::build_LinearSystem(Sophus::SE3 &model)
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

            H_.noalias()    += J * J.transpose() * weight;
            Jres_.noalias() -= J * res * weight;

            // if(display_)
            //     resimg_.at<float>((int) v_cur + y - patch_halfsize_, (int) u_cur + x - patch_halfsize_) =
            //             res / 255.0;
        }

        // std::cout<<"SW Hessian matrix: "<<std::endl<<H_.matrix()<<std::endl;
        
        return res;
    }

} // namespace ORB_SLAM2