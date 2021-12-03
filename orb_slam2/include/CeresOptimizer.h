/*
 * @Author: kinggreat24
 * @Date: 2020-11-30 09:26:31
 * @LastEditTime: 2021-04-01 10:43:13
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/include/CeresOptimizer.h
 * @可以输入预定的版权声明、个性签名、空行等
 */
#ifndef CERES_OPTIMIZER_H
#define CERES_OPTIMIZER_H

#include <iostream>
#include <vector>
#include <chrono>

#include "Common.h"

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "CeresFactor.hpp"
#include "Converter.h"

namespace ORB_SLAM2{

class CeresOptimizer{
public:

int static PoseOptimization(Frame* pF,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalCornerMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalSurfMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurCornerScan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurSurfScan,
    int max_iteration = 3
);

float static PoseOptimization(Eigen::Matrix4d& transform_wc,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalCornerMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalSurfMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurCornerScan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurSurfScan,
    int max_iteration=3);

int static PoseOptimization(KeyFrame* pKF, Map *pMap);
int static Frame2FramePoseOptimization(Frame* pF);
static int CurveFitting();

};

}



#endif//CERES_OPTIMIZER_H