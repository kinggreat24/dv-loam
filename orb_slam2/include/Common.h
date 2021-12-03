/*
 * @Author: kinggreat24
 * @Date: 2021-03-31 21:19:32
 * @LastEditTime: 2021-08-18 16:21:18
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/include/Common.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <eigen3/Eigen/Core>

#include <unistd.h>
#include <iostream>
#include <stdarg.h>
#include <cstdio>
#include <string.h>
#include <stdlib.h>

// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h> //条件滤波
#include <pcl/filters/voxel_grid.h>          //体素滤波器头文件
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波

#include "lidar_sparse_align/WeightFunction.h"

#ifdef USE_GPU
#include "darknet_detector/darknet_detector.h"
#endif //

using namespace std;

namespace ORB_SLAM2
{

    typedef Eigen::Matrix<double, 4, 1> Vector4d;
    typedef Eigen::Matrix<float, 6, 1> Vector6;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<float, 2, 6> Matrix2x6;
    typedef Eigen::Matrix<double, 4, 4> Matrix4d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;

    typedef pcl::PointXYZI PointType;
    typedef pcl::PointXYZINormal PointNormal;
    typedef std::vector<Eigen::Vector3d> PL_VEC;

    typedef struct _tracker_t tracker_t;
    struct _tracker_t
    {
        // 金字塔层数以及最大迭代次数
        int levels;
        int min_level;
        int max_level;
        int max_iteration;

        // 直接法权重函数
        bool use_weight_scale = true;
        string scale_estimator;
        string weight_function;
        ScaleEstimatorType scale_estimator_type;
        WeightFunctionType weight_function_type;


        // 间接法权重函数
        bool feature_use_weight_scale = true;
        string feature_scale_estimator;
        string feature_weight_function;
        ScaleEstimatorType feature_scale_estimator_type;
        WeightFunctionType feature_weight_function_type;


        /****************             直接法权重函数            ******************/
        void set_scale_estimator_type()
        {
            if (!scale_estimator.compare("None"))
                use_weight_scale = false;
            if (!scale_estimator.compare("TDistributionScale"))
                scale_estimator_type = ScaleEstimatorType::TDistributionScale;

            cerr << "ScaleType : " << static_cast<int>(scale_estimator_type);
        }

        void set_weight_function_type()
        {
            if (!weight_function.compare("TDistributionWeight"))
                weight_function_type = WeightFunctionType::TDistributionWeight;
        }




        /****************             间接法权重函数            ******************/
        void set_feature_scale_estimator_type()
        {
            if (!feature_scale_estimator.compare("None"))
                feature_use_weight_scale = false;
            // if (!feature_scale_estimator.compare("GaussianDistributionScale"))
            //     feature_scale_estimator_type = ScaleEstimatorType::GaussianDristributionScale;

            cerr << "Feature ScaleType : " << static_cast<int>(scale_estimator_type);
        }

        void set_feature_weight_function_type()
        {
            if (!feature_weight_function.compare("HuberWeight"))
                feature_weight_function_type = WeightFunctionType::HuberWeight;
        }
    };

    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).
    // The order is roll pitch yaw
    inline Eigen::Vector3d rotationMatrixToEulerAnglesRPY(const Eigen::Matrix3d &R)
    {
        // assert(isRotationMatrix(R));
        float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

        bool singular = sy < 1e-6; // If

        float x, y, z;
        if (!singular)
        {
            x = atan2(R(2, 1), R(2, 2));
            y = atan2(-R(2, 0), sy);
            z = atan2(R(1, 0), R(0, 0));
        }
        else
        {
            x = atan2(-R(1, 2), R(1, 1));
            y = atan2(-R(2, 0), sy);
            z = 0;
        }
        return Eigen::Vector3d(x, y, z);
    }

    inline Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta_rpy, bool use_axis = false)
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        if (!use_axis)
        {
            // Calculate rotation about x axis
            Eigen::Matrix3d R_x = Eigen::Matrix3d::Identity();
            R_x << 1, 0, 0,
                0, cos(theta_rpy[0]), -sin(theta_rpy[0]),
                0, sin(theta_rpy[0]), cos(theta_rpy[0]);

            // Calculate rotation about y axis
            Eigen::Matrix3d R_y = Eigen::Matrix3d::Identity();
            R_y << cos(theta_rpy[1]), 0, sin(theta_rpy[1]),
                0, 1, 0,
                -sin(theta_rpy[1]), 0, cos(theta_rpy[1]);

            // Calculate rotation about z axis
            Eigen::Matrix3d R_z = Eigen::Matrix3d::Identity();
            R_z << cos(theta_rpy[2]), -sin(theta_rpy[2]), 0,
                sin(theta_rpy[2]), cos(theta_rpy[2]), 0,
                0, 0, 1;

            // Combined rotation matrix
            R = R_z * R_y * R_x;
        }
        else
        {
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(theta_rpy[0], Eigen::Matrix<double, 3, 1>::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(theta_rpy[1], Eigen::Matrix<double, 3, 1>::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(theta_rpy[2], Eigen::Matrix<double, 3, 1>::UnitZ()));

            R = yawAngle * pitchAngle * rollAngle;
        }

        return R;
    }

    inline Eigen::Matrix3f skew_symmetric(Eigen::Vector3f v)
    {
        Eigen::Matrix3f S;
        S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        return S;
    }

    inline Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v)
    {
        Eigen::Matrix3d S;
        S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        return S;
    }

    inline cv::Mat SkewSymmetricMatrix(const cv::Mat &v)
    {
        return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2), 0, -v.at<float>(0),
                -v.at<float>(1), v.at<float>(0), 0);
    }

    inline cv::Scalar RandomColor(int64 seed)
    {
        cv::RNG rng(seed);
        int icolor = (unsigned int)rng;
        return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
    }

    //将上一帧的点投影到当前帧
    inline void TransformToEnd(const PointType *point_last, const Eigen::Quaterniond &q_cur_last, const Eigen::Vector3d &t_cur_last, PointType *point_cur)
    {
        Eigen::Vector3d pl(point_last->x, point_last->y, point_last->z);
        Eigen::Vector3d pc = q_cur_last.toRotationMatrix() * pl + t_cur_last;
        point_cur->x = pc[0];
        point_cur->y = pc[1];
        point_cur->z = pc[2];
        point_cur->intensity = point_last->intensity;
    }

    //将当前帧的点投影到上一帧
    inline void TransformToStart(const PointType &point_cur, const Eigen::Quaterniond &q_last_cur, const Eigen::Vector3d &t_last_cur, PointType &point_last)
    {
        Eigen::Vector3d pc(point_cur.x, point_cur.y, point_cur.z);
        Eigen::Vector3d pl = q_last_cur.toRotationMatrix() * pc + t_last_cur;
        point_last.x = pl[0];
        point_last.y = pl[1];
        point_last.z = pl[2];
        point_last.intensity = point_cur.intensity;
    }

    inline void TransformPoint(const PointType *point_cam, const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc, PointType *point_world)
    {
        Eigen::Vector3d pc(point_cam->x, point_cam->y, point_cam->z);
        Eigen::Vector3d pw = q_wc.toRotationMatrix() * pc + t_wc;
        point_world->x = pw[0];
        point_world->y = pw[1];
        point_world->z = pw[2];
        point_world->intensity = point_cam->intensity;
    }

    inline void pointAssociateToMap(
        const pcl::PointXYZI *pointOri,
        const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc,
        pcl::PointXYZI *pointSel)
    {
        Eigen::Vector3d pointOriVec(pointOri->x, pointOri->y, pointOri->z);
        Eigen::Vector3d pointSelVec = q_wc.toRotationMatrix() * pointOriVec + t_wc;
        pointSel->x = pointSelVec[0];
        pointSel->y = pointSelVec[1];
        pointSel->z = pointSelVec[2];
        pointSel->intensity = pointOri->intensity;
    }

    inline double compute_inlier_residual_threshold_corner(std::vector<double> residuals, double ratio)
    {
        std::set<double> dis_vec;
        for (size_t i = 0; i < (size_t)(residuals.size() / 3); i++)
        {
            dis_vec.insert(fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) + fabs(residuals[3 * i + 2]));
        }
        return *(std::next(dis_vec.begin(), (int)((ratio)*dis_vec.size())));
    }

    inline double compute_inlier_residual_threshold_surf(std::vector<double> residuals, double ratio)
    {
        return *(std::next(residuals.begin(), (int)((ratio)*residuals.size())));
    }

    //! Bounding box of the detected object.
    typedef struct
    {
        float x, y, w, h, prob;
        int num, Class;
        std::string label;
        void print_info()
        {
            std::cout << "label: " << label
                      << " class id: " << Class
                      << " prob: " << prob
                      << " x: " << x << " y: " << y << " w: " << w << " h: " << h << std::endl;
        }

        bool isInsideBox(int u, int v)
        {
            if (u >= x && u <= x + w && v >= y && v <= y + h)
                return true;
            else
                return false;
        }
    } ObjBox_;

    typedef struct Box_
    {
        float x, y, w, h;
    } Box;

    // IOU计算
    inline float overlap(float x1, float w1, float x2, float w2)
    {
        float l1 = x1 - w1 / 2;
        float l2 = x2 - w2 / 2;
        float left = l1 > l2 ? l1 : l2;
        float r1 = x1 + w1 / 2;
        float r2 = x2 + w2 / 2;
        float right = r1 < r2 ? r1 : r2;
        return right - left;
    }

    inline float box_intersection(Box a, Box b)
    {
        float w = overlap(a.x, a.w, b.x, b.w);
        float h = overlap(a.y, a.h, b.y, b.h);
        if (w < 0 || h < 0)
            return 0;
        float area = w * h;
        return area;
    }

    inline float box_union(Box a, Box b)
    {
        float i = box_intersection(a, b);
        float u = a.w * a.h + b.w * b.h - i;
        return u;
    }

    inline float box_iou(Box a, Box b)
    {
        return box_intersection(a, b) / box_union(a, b);
    }

    inline float obj_iou(Box a, Box b)
    {
        return box_intersection(a, b) / (b.w * b.h);
    }

    enum Color
    {
        BLACK = 30,
        RED = 31,
        GREEN = 32,
        BROWN = 33,
        BLUE = 34,
        MAGENTA = 35,
        CYAN = 36,
        GREY = 37,
        LRED = 41,
        LGREEN = 42,
        YELLOW = 43,
        LBLUE = 44,
        LMAGENTA = 45,
        LCYAN = 46,
        WHITE = 47
    };

    inline void set_color(int fd, Color color)
    {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "\x1b[%d%sm",
                 color >= LRED ? (color - 10) : color,
                 color >= LRED ? ";1" : "");
        write(fd, buffer, strlen(buffer));
    }

    inline void reset_color(int fd)
    {
        const char *s = "\x1b[0m";
        write(fd, s, strlen(s));
    }

    inline double vector_stdv_mad(vector<double> residues_in)
    {
        // remove elements which lower than 0, those elements are invalid.
        vector<double> residues;
        for(size_t i=0;i<residues_in.size();i++)
        {
            if(residues_in.at(i) < 0)
                continue;

            residues.push_back(residues_in.at(i));
        }
        
        if (residues.size() != 0)
        {
            // Return the standard deviation of vector with MAD estimation
            int n_samples = residues.size();
            sort(residues.begin(), residues.end());
            double median = residues[n_samples / 2];
            for (int i = 0; i < n_samples; i++)
                residues[i] = fabsf(residues[i] - median);
            sort(residues.begin(), residues.end());
            double MAD = residues[n_samples / 2];
            return 1.4826 * MAD;
        }
        else
            return 0.0;
    }

    inline double vector_stdv_mad(vector<double> residues_in, double &median)
    {
        // remove elements which lower than 0, those elements are invalid.
        std::vector<double> residues;
        for(size_t i=0;i<residues_in.size();i++)
        {
            if(residues_in.at(i) < 0)
                continue;

            residues.push_back(residues_in.at(i));
        }

        if (residues.size() != 0)
        {
            // Return the standard deviation of vector with MAD estimation
            int n_samples = residues.size();
            sort(residues.begin(), residues.end());
            median = residues[n_samples / 2];
            for (int i = 0; i < n_samples; i++)
                residues[i] = fabsf(residues[i] - median);
            sort(residues.begin(), residues.end());
            double MAD = residues[n_samples / 2];
            return 1.4826 * MAD;
        }
        else
            return 0.0;
    }

} //end of namespace ORB_SLAM

#endif //