// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"


SWOdomEstimationClass::SWOdomEstimationClass()
{
}


void SWOdomEstimationClass::init(double map_resolution)
{
    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    // odom = Eigen::Isometry3d::Identity();
    // last_odom = Eigen::Isometry3d::Identity();
    optimization_count = 2;
}

void SWOdomEstimationClass::initMapWithPoints(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in)
{
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap   += *surf_in;
    optimization_count = 12;
}

void SWOdomEstimationClass::updatePointsToMap(Eigen::Matrix4d& Twc,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in)
{
    if (optimization_count > 3)
        optimization_count--;


    // 设置初值
    q_w_curr = Eigen::Quaterniond(Twc.block<3,3>(0,0));
    q_w_curr.normalize();
    t_w_curr = Twc.block<3,1>(0,3);


    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in, downsampledEdgeCloud, surf_in, downsampledSurfCloud);
    
    // 当前帧与地图对齐
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if (laserCloudCornerMap->points.size() > 10 && laserCloudSurfMap->points.size() > 50)
    {
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

            addEdgeCostFactor(downsampledEdgeCloud, laserCloudCornerMap, problem, loss_function);
            addSurfCostFactor(downsampledSurfCloud, laserCloudSurfMap, problem, loss_function);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 10;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients    = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }
    }
    else
    {
        printf("not enough points in map to associate, map error");
    }

    // 更新结果
    Twc.block<3,3>(0,0) = q_w_curr.toRotationMatrix();
    Twc.block<3,1>(0,3) = t_w_curr;

    //将当前帧点云添加到局部地图中
    addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);
}

void SWOdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void SWOdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out)
{
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);
}

void SWOdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem, ceres::LossFunction *loss_function)
{
    int corner_num = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;
            }
        }
    }
    if (corner_num < 20)
    {
        printf("not enough correct points");
    }

    // std::cout << "corner_measurements: " << corner_num << std::endl;
}

void SWOdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem, ceres::LossFunction *loss_function)
{
    int surf_num = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {

            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }
    }
    if (surf_num < 20)
    {
        printf("not enough correct points");
    }

    // std::cout << "surf_measurements: " << surf_num << std::endl;
}

void SWOdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud)
{

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp);
    }

    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        laserCloudSurfMap->push_back(point_temp);
    }

    double x_min = +t_w_curr.x() - 100;
    double y_min = +t_w_curr.y() - 100;
    double z_min = +t_w_curr.z() - 100;
    double x_max = +t_w_curr.x() + 100;
    double y_max = +t_w_curr.y() + 100;
    double z_max = +t_w_curr.z() + 100;

    // printf("box size : %f,%f,%f,%f,%f,%f \r\n", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);


    // static int cnt = 0;
    // if(cnt++ % 10 == 0)
    // {
    //     char static_pc_name[128]={0};
    //     sprintf(static_pc_name,"/home/kinggreat24/pc/%d_map_corner.pcd",cnt);
    //     pcl::io::savePCDFileASCII(static_pc_name, *laserCloudCornerMap);//将点云保存到PCD文件中

    //     sprintf(static_pc_name,"/home/kinggreat24/pc/%d_scan_corner.pcd",cnt);
    //     pcl::io::savePCDFileASCII(static_pc_name, *downsampledEdgeCloud);//将点云保存到PCD文件中
    // }
    // std::cout<<"addPointsToMap over"<<std::endl;
}

void SWOdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudMap)
{

    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}
