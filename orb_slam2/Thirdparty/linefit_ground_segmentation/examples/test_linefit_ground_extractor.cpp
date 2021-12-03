/*
 * @Author: your name
 * @Date: 2020-05-06 10:27:16
 * @LastEditTime: 2021-02-22 22:34:26
 * @LastEditors: kinggreat24
 * @Description: In User Settings Edit
 * @FilePath: /direct_lidar_alignment/Thirdparty/linefit_ground_segmentation/examples/test_linefit_ground_extractor.cpp
 */
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "ground_segmentation/ground_segmentation.h"

using namespace linefit_ground_segmentation;
using namespace std;

typedef pcl::PointXYZI PointType;


// std::string data_path = std::string("/media/kinggreat24/Samsung_T5/data/kitti_data_full/odometry/unzip/data/dataset/sequences/00");
std::string setting_file = std::string("/home/kinggreat24/direct_lidar_align_ws/src/direct_lidar_alignment/Thirdparty/linefit_ground_segmentation/examples/segmentation_params.yaml");

Eigen::Matrix<float,3,4> extrinsicMatrix;

float fx_, fy_, cx_, cy_;
float width_,height_;
float mMinDepth = 4;
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
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
    string strPrefixLabel = strPathToSequence + "/label/";
    string strPrefixLidar = strPathToSequence + "/velodyne/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageLabel.resize(nTimes);
    vstrLidarFile.resize(nTimes);


    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixImg + ss.str() + ".png";
        vstrImageLabel[i] = strPrefixLabel + ss.str() + ".png";
        vstrLidarFile[i] = strPrefixLidar + ss.str() + ".bin";
    }
}



int ReadPointCloud(const std::string& file, pcl::PointCloud<PointType>::Ptr outpointcloud, bool isBinary)
{
    outpointcloud->height = 1;
    if(isBinary)
    {
        // load point cloud
        std::fstream input(file.c_str(), std::ios::in | std::ios::binary);
        if (!input.good()) {
            std::cerr << "Could not read file: " << file << std::endl;
            exit(EXIT_FAILURE);
        }
        //LOG(INFO)<<"Read: "<<file<<std::endl;

        for (int i = 0; input.good() && !input.eof(); i++) {
            pcl::PointXYZI point;
            input.read((char *)&point.x, 3 * sizeof(float));
            input.read((char *)&point.intensity, sizeof(float));
            
            //remove all points behind image plane (approximation)
            if (point.x < mMinDepth)
                continue;
            outpointcloud->points.push_back(point);
        }
        outpointcloud->width = outpointcloud->points.size();
    }
   
    return outpointcloud->points.size();
}

inline Eigen::Vector2f xyz_to_uv(const Eigen::Vector3f xyz)
{
    Eigen::Vector2f uv;
    uv[0] = fx_ * xyz.x() / xyz.z() + cx_;
    uv[1] = fy_ * xyz.y() / xyz.z() + cy_;
    return uv;
}


inline bool is_in_image(const Eigen::Vector2f& uv, const float boundary)
{
    int u = static_cast<int> (uv(0));
    int v = static_cast<int> (uv(1));

    if (u > 0 + boundary && u < static_cast<int> (float(width_)) - boundary && v > 0 + boundary && v < static_cast<int> (float(height_)) - boundary) {
        return true;
    }
    else {
        return false;
    }
}

void point_sampling(const cv::Mat& img, const pcl::PointCloud<pcl::PointXYZ>& point_input,pcl::PointCloud<pcl::PointXYZ>& point_output)
{
    int num_bucket_size = 10;
   
    vector<pair<float, pcl::PointXYZ> > mag_point_bucket;
    mag_point_bucket.reserve(num_bucket_size);

    int num_out_points = 0;

    for(auto iter = point_input.begin(); iter != point_input.end(); ++iter) {
    
        Eigen::Vector3f xyz_ref ( iter->x, iter->y, iter->z );
        Eigen::Vector2f uv = xyz_to_uv(xyz_ref);

        if (is_in_image(uv, 4) && iter->z > 0) 
        {

            int u = static_cast<int> (uv(0));
            int v = static_cast<int> (uv(1));

            float dx = 0.5f * (img.at<float> (v, u+1) - img.at<float> (v, u-1));
            float dy = 0.5f * (img.at<float> (v+1, u) - img.at<float> (v-1, u));

            std::pair<float, pcl::PointXYZ> mag_point;
            mag_point = make_pair((dx*dx+dy*dy), (*iter));

            mag_point_bucket.push_back(mag_point);
            if(mag_point_bucket.size() == num_bucket_size)
            {

                float max = -1;
                int idx;
                for(int i=0; i<mag_point_bucket.size(); ++i) 
                {
                    if (mag_point_bucket[i].first > max) 
                    {
                        max = mag_point_bucket[i].first;
                        idx = i;
                    }

                }

                if(max > (6.25/ (255.0*255.0)))  // 16.25
                    point_output.push_back(mag_point_bucket[idx].second);

                mag_point_bucket.clear();
            }
        }
        else 
        {
            num_out_points++;
        }

    }
    // std::cout<<"sampling point size: "<<point_output.size()<<std::endl;
    assert(num_out_points != 0 && "Points must be projected at all pyramid levels.");
}


void ShowGroundPointClouds(const pcl::PointCloud<pcl::PointXYZ>& pc,  const cv::Mat& image_in, cv::Mat& image_out, size_t num_level)
{
    if(image_in.channels() == 1) 
    {
        cvtColor(image_in, image_out, cv::COLOR_GRAY2BGR);
    }
    else 
    {
        image_in.copyTo(image_out);
    }
    

    const float scale = 1.0f/(1<<num_level);

    int n = 0;
    for(auto iter=pc.begin(); iter!=pc.end(); ++iter) 
    {
        n++;
        if(n%3 != 0) continue;

        if(iter->z < 0)
            continue;

        Eigen::Vector3f xyz_ref ( iter->x, iter->y, iter->z );
        Eigen::Vector2f uv_ref;
        uv_ref.noalias() = xyz_to_uv(xyz_ref) * scale;

        const float u_ref_f = uv_ref(0);
        const float v_ref_f = uv_ref(1);
        const int u_ref_i = static_cast<int> (u_ref_f);
        const int v_ref_i = static_cast<int> (v_ref_f);

        float v_min = 1.0;    float v_max = 50.0;    float dv = v_max - v_min;
        float v = xyz_ref(2);
        float r = 1.0; float g = 1.0; float b = 1.0;
        if (v < v_min)   v = v_min;
        if (v > v_max)   v = v_max;

        if(v < v_min + 0.25*dv) {
            r = 0.0;
            g = 4*(v - v_min) / dv;
        }
        else if (v < (v_min + 0.5 * dv)) {
            r = 0.0;
            b = 1 + 4*(v_min + 0.25 * dv - v) / dv;
        } else if (v < (v_min + 0.75 * dv)) {
            r =4 * (v - v_min - 0.5 * dv) / dv;
            b = 0.0;
        } else {
            g = 1 + 4*(v_min + 0.75 * dv - v) / dv;
            b = 0.0;
        }

        cv::circle(image_out, cv::Point(u_ref_i, v_ref_i), 3.5, cv::Scalar( r*255, g*255, b*255 ), -1);
    }
    // image_out.convertTo(image_out, CV_8UC3, 255);
}



int main(int argc, char** argv)
{
    std::cout<<"Load configurations"<<std::endl;

    cv::FileStorage f_hw_settings(setting_file, cv::FileStorage::READ);
    GroundSegmentationParams params;
    
    params.n_threads         = f_hw_settings["n_threads"];
    params.n_bins            = f_hw_settings["n_bins"];       
    params.n_segments        = f_hw_settings["n_segments"];
    params.max_dist_to_line  = f_hw_settings["max_dist_to_line"];
    params.sensor_height     = f_hw_settings["sensor_height"];
    params.max_slope         = f_hw_settings["max_slope"];
    params.long_threshold    = f_hw_settings["long_threshold"];
    params.max_long_height   = f_hw_settings["max_long_height"];
    params.max_start_height  = f_hw_settings["max_start_height"];
    params.line_search_angle = f_hw_settings["line_search_angle"];  
    
    float r_min              = f_hw_settings["r_min"];
    float r_max              = f_hw_settings["r_max"];
    float max_fit_error      = f_hw_settings["max_fit_error"];  
    params.r_min_square      = r_min * r_min;
    params.r_max_square      = r_max * r_max;
    params.max_error_square  = max_fit_error * max_fit_error;
    
    // Retrieve paths to images
    std::string data_path = std::string(f_hw_settings["data_path"]);
    vector<string> vstrImageGray;
    vector<string> vstrImageLabel;
    vector<string> vstrLidar;
    vector<double> vTimestamps;
    LoadImages(data_path, vstrImageGray, vstrImageLabel,vstrLidar, vTimestamps);
    const int nImages = vstrImageGray.size();
    
    //load camera information
    width_  =  f_hw_settings["Camera.width"];
    height_ =  f_hw_settings["Camera.height"];
    fx_ = f_hw_settings["Camera.fx"];  
    fy_ = f_hw_settings["Camera.fy"];
    cx_ = f_hw_settings["Camera.cx"];
    cy_ = f_hw_settings["Camera.cy"];

    // load camera lidar extrinsic matrix
    cv::Mat T;
    Eigen::Matrix4f extrinsic = Eigen::Matrix4f::Identity();
    int need_inverse    =  f_hw_settings["extrinsicMatrix.need_inverse"];
    f_hw_settings["extrinsicMatrix"] >> T;
    cv::cv2eigen(T, extrinsic);
    if(need_inverse)
    {
        Eigen::Matrix3f R = extrinsic.topLeftCorner(3,3);
        Eigen::Vector3f t = extrinsic.topRightCorner(3,1);

        extrinsic.topLeftCorner(3,3)  =  R.transpose();
        extrinsic.topRightCorner(3,1) = -R.transpose()*t;
    }

    // extrinsicMatrix <<
    //        0.0004276802385584, -0.9999672484946, -0.008084491683471, -0.01198459927713,
    //       -0.007210626507497,  0.008081198471645, -0.9999413164504, -0.05403984729748,
    //        0.9999738645903,  0.0004859485810390, -0.007206933692422, -0.2921968648686;
    

    // Eigen::Matrix4f extrinsic = Eigen::Matrix4f::Identity();
    // extrinsic.block<3,4>(0,0) = extrinsicMatrix;

    cv::Mat imGray;
    pcl::PointCloud<PointType>::Ptr pc_full;
    for(int ni=0;ni<nImages;ni++)
    {
        pc_full.reset(new pcl::PointCloud<PointType>());
        imGray = cv::imread(vstrImageGray[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imGray.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageGray[ni]) << endl;
            return 1;
        }

        int points = ReadPointCloud(vstrLidar[ni],pc_full,true);


        pcl::PointCloud<pcl::PointXYZ> cur_pc;
        pcl::copyPointCloud(*pc_full,cur_pc);

        GroundSegmentation segmenter(params);
        std::vector<int> labels;
        segmenter.segment(cur_pc,&labels);

        pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
        for (size_t i = 0; i < cur_pc.size(); ++i) 
        {
            if (labels[i] == 1) 
                ground_cloud.push_back(cur_pc[i]);
            else 
                obstacle_cloud.push_back(cur_pc[i]);
        }

        //将地面点投影到图像上
        pcl::transformPointCloud(ground_cloud, ground_cloud, extrinsic);
        
        // pcl::PointCloud<pcl::PointXYZ> sample_pointcloud;
        // point_sampling(imGray,ground_cloud,sample_pointcloud);
        
        cv::Mat img_with_ground_points;
        ShowGroundPointClouds(ground_cloud, imGray, img_with_ground_points, 0);
        cv::imshow("ground_points",img_with_ground_points);

        // cv::Mat img_with_obstacle_points;
        // ShowGroundPointClouds(obstacle_cloud, imGray, img_with_obstacle_points, 0);
        // cv::imshow("obstacle_points",img_with_obstacle_points);
        cv::waitKey(1);

    }

    
    // pcl::PLYWriter writer;
    // writer.write("./full.ply", *pc_full);
    // writer.write("./cornerPointsSharp.ply", *cornerPointsSharp);
    // writer.write("./cornerPointsLessSharp.ply", *cornerPointsLessSharp);
    // writer.write("./surfPointsFlat.ply", *surfPointsFlat);
    // writer.write("./surfPointsLessFlat.ply", *surfPointsLessFlat);

    return 0;
}
