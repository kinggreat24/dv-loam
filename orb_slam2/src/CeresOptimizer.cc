/*
 * @Author: kinggreat24
 * @Date: 2020-11-30 09:28:20
 * @LastEditTime: 2021-04-06 15:20:48
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/src/CeresOptimizer.cc
 * @可以输入预定的版权声明、个性签名、空行等
 */
#include "CeresOptimizer.h"
#include "Common.h"

using namespace std;

// #define USE_VISUAL_FEATURES 0

namespace ORB_SLAM2
{

int CeresOptimizer::PoseOptimization(Frame* pFrame, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalCornerMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalSurfMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurCornerScan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurSurfScan,
    int max_iteration)
{
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0}; //orentation translation
    Eigen::Map<Eigen::Quaterniond> m_q_w_curr(parameters);
    Eigen::Map<Eigen::Vector3d> m_t_w_curr(parameters + 4);

    Eigen::Matrix4d Twc = Converter::toMatrix4d(pFrame->mTcw).inverse();
    Eigen::Quaterniond q(Twc.topLeftCorner<3, 3>(0, 0));
    q.normalize();
    Eigen::Vector3d t = Twc.block<3, 1>(0, 3);
    // std::cout<<"Frame id: "<<pFrame->mnId<<std::endl;
    // std::cout<<"Before optimization Twc: "<<std::endl<<Twc<<std::endl;

    //6DOF优化初值
    parameters[0] = q.x();
    parameters[1] = q.y();
    parameters[2] = q.z();
    parameters[3] = q.w();
    parameters[4] = t[0];
    parameters[5] = t[1];
    parameters[6] = t[2];

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    PointType pointOri, pointSel, pointProj, coeff;

    const int laserCloudCornerLastDSNum = pCurCornerScan->size();
    const int laserCloudSurfLastDSNum   = pCurSurfScan->size();

    ceres::LinearSolverType slover_type = ceres::DENSE_SCHUR; // SPARSE_NORMAL_CHOLESKY | DENSE_QR | DENSE_SCHUR
    int m_para_cere_max_iterations      = 100;
    int m_para_cere_prerun_times        = 2;

    double m_inlier_ratio = 0.80;
    double m_inliner_dis = 0.02;
    double m_inliner_dis_visual = 5.0;
    double m_inlier_threshold_corner;
    double m_inlier_threshold_surf;
    double m_inlier_threshold_points;

    // 6-DOF 优化
    if (pLocalCornerMap->size() > 10 && pLocalSurfMap->size() > 10)
    {
        pcl::KdTreeFLANN<PointType> m_kdtree_corner_from_map;
        pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map;
        m_kdtree_corner_from_map.setInputCloud(pLocalCornerMap);
        m_kdtree_surf_from_map.setInputCloud(pLocalSurfMap);

        for (int iterCount = 0; iterCount < max_iteration; iterCount++)
        {
            ceres::LossFunction *loss_function               = new ceres::HuberLoss(0.1);
            ceres::LossFunction *visual_loss_function        = new ceres::HuberLoss(sqrt(5.991));
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::ResidualBlockId block_id;
            ceres::Problem problem(problem_options);
            std::vector<ceres::ResidualBlockId> mappoints_residual_block_ids;
            std::vector<ceres::ResidualBlockId> corner_residual_block_ids;
            std::vector<ceres::ResidualBlockId> surf_residual_block_ids;

            problem.AddParameterBlock(parameters, 4, q_parameterization);
            problem.AddParameterBlock(parameters + 4, 3);
        
        
        #ifdef USE_VISUAL_FEATURES
            //Add MapPoints
            int visual_num = 0;
            std::vector<Eigen::Vector3d> mappoints_pos_w;
            for (int i=0;i<pFrame->N;i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if(pMP)
                {
                    if(pFrame->mvbOutlier[i])
                        continue;
                    Eigen::Vector3d  pos_w = Converter::toVector3d(pMP->GetWorldPos());
                    
                    // Monocular observation
                    cv::KeyPoint kpUn = pFrame->mvKeysUn[i];                
                    ceres::CostFunction *cost_function = SnavelyReprojectionFactorPoseOnly::Create(pos_w,kpUn.pt.x, kpUn.pt.y,pFrame->mpPinholeCamera);
                    block_id = problem.AddResidualBlock(cost_function, visual_loss_function, parameters, parameters + 4);
                    mappoints_residual_block_ids.push_back(block_id);

                    visual_num++;
                }
            }
        #endif //USE_VISUAL_FEATURES

        
            // Add Corner Points
            int corner_num = 0;
            int corner_num_rejected = 0;
            for (int i = 0; i < laserCloudCornerLastDSNum; i++)
            {
                pointOri = pCurCornerScan->points[i];
                pointAssociateToMap(&pointOri, m_q_w_curr, m_t_w_curr, &pointSel);
                m_kdtree_corner_from_map.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                if (pointSearchSqDis[4] < 1.0)
                {
                    std::vector<Eigen::Vector3d> nearCorners;
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Vector3d tmp(pLocalCornerMap->points[pointSearchInd[j]].x,
                                            pLocalCornerMap->points[pointSearchInd[j]].y,
                                            pLocalCornerMap->points[pointSearchInd[j]].z);
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

                    // if is indeed line feature
                    // note Eigen library sort eigenvalues in increasing order
                    Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    {
                        Eigen::Vector3d point_on_line = center;
                        Eigen::Vector3d point_a, point_b;
                        point_a =  0.1 * unit_direction + point_on_line;
                        point_b = -0.1 * unit_direction + point_on_line;

                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                        block_id = problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                        corner_residual_block_ids.push_back(block_id);
                        corner_num++;
                    }
                    else
                    {
                        corner_num_rejected++;
                    }
                }
            }

            // Add Surf Points
            int surf_num = 0;
            int surf_rejected_num = 0;
            for (int i = 0; i < laserCloudSurfLastDSNum; i++)
            {
                pointOri = pCurSurfScan->points[i];
                pointAssociateToMap(&pointOri, m_q_w_curr, m_t_w_curr, &pointSel);
                m_kdtree_surf_from_map.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < 1.0)
                {

                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = pLocalSurfMap->points[pointSearchInd[j]].x;
                        matA0(j, 1) = pLocalSurfMap->points[pointSearchInd[j]].y;
                        matA0(j, 2) = pLocalSurfMap->points[pointSearchInd[j]].z;
                        //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                    }
                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * pLocalSurfMap->points[pointSearchInd[j]].x +
                                 norm(1) * pLocalSurfMap->points[pointSearchInd[j]].y +
                                 norm(2) * pLocalSurfMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    if (planeValid)
                    {
                        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                        block_id = problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                        surf_residual_block_ids.push_back(block_id);
                        surf_num++;
                    }
                    else
                    {
                        surf_rejected_num++;
                    }
                }
            }

            //优化
            ceres::Solver::Options options;
            options.linear_solver_type = /*ceres::DENSE_QR*/ slover_type;
            options.max_num_iterations = /*100*/ m_para_cere_prerun_times;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-6;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // 计算边缘的残差,剔除错误的匹配
            ceres::Problem::EvaluateOptions eval_options;
            eval_options.residual_blocks = corner_residual_block_ids;
            double corner_total_cost = 0.0;
            std::vector<double> corner_residuals; //每个观测的误差
            problem.Evaluate(eval_options, &corner_total_cost, &corner_residuals, nullptr, nullptr);

            // 计算平坦点的残差,剔除错误的匹配
            eval_options.residual_blocks = surf_residual_block_ids;
            double surf_total_cost = 0.0;
            std::vector<double> surf_residuals; //每个观测的误差
            problem.Evaluate(eval_options, &surf_total_cost, &surf_residuals, nullptr, nullptr);


        #ifdef USE_VISUAL_FEATURES
            //计算视觉特征点重投影残差，剔除错误的匹配
            eval_options.residual_blocks = mappoints_residual_block_ids;
            double mappoints_total_cost = 0.0;
            std::vector<double> mappoints_residuals; //每个观测的误差
            problem.Evaluate(eval_options, &mappoints_total_cost, &mappoints_residuals, nullptr, nullptr);
        #endif//

            //边缘点
            std::vector<ceres::ResidualBlockId> corner_residual_block_ids_temp;
            corner_residual_block_ids_temp.reserve(corner_residual_block_ids.size());
            double m_inliner_ratio_threshold_corner = compute_inlier_residual_threshold_corner(corner_residuals, m_inlier_ratio);
            m_inlier_threshold_corner = std::max(m_inliner_dis, m_inliner_ratio_threshold_corner);

            //平坦点
            std::vector<ceres::ResidualBlockId> surf_residual_block_ids_temp;
            surf_residual_block_ids_temp.reserve(surf_residual_block_ids.size());
            double m_inliner_ratio_threshold_surf = compute_inlier_residual_threshold_surf(surf_residuals, m_inlier_ratio);
            m_inlier_threshold_surf = std::max(m_inliner_dis, m_inliner_ratio_threshold_surf);


        #ifdef USE_VISUAL_FEATURES
            std::vector<ceres::ResidualBlockId> mappoints_residual_block_ids_temp;
            mappoints_residual_block_ids_temp.reserve(mappoints_residual_block_ids.size());
            double m_inliner_ratio_threshold_mappoints = compute_inlier_residual_threshold_mappoints(mappoints_residuals, m_inlier_ratio);
            m_inlier_threshold_points = std::max(m_inliner_dis_visual, m_inliner_ratio_threshold_mappoints);
        #endif//

            //剔除误差较大的边缘激光点
            for (unsigned int i = 0; i < corner_residual_block_ids.size(); i++)
            {
                if ((fabs(corner_residuals[3 * i + 0]) + fabs(corner_residuals[3 * i + 1]) + fabs(corner_residuals[3 * i + 2])) > m_inlier_threshold_corner) // std::min( 1.0, 10 * avr_cost )
                {
                    //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                    problem.RemoveResidualBlock(corner_residual_block_ids[i]);
                }
                else
                {
                    corner_residual_block_ids_temp.push_back(corner_residual_block_ids[i]);
                }
            }

            //剔除误差较大的平坦点激光点
            for (unsigned int i = 0; i < surf_residual_block_ids.size(); i++)
            {
                if (fabs(surf_residuals[i]) > m_inlier_threshold_surf) // std::min( 1.0, 10 * avr_cost )
                {
                    //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                    problem.RemoveResidualBlock(surf_residual_block_ids[i]);
                }
                else
                {
                    surf_residual_block_ids_temp.push_back(surf_residual_block_ids[i]);
                }
            }

        #ifdef USE_VISUAL_FEATURES
            //剔除误差较大的地图点
            for (unsigned int i = 0; i < mappoints_residual_block_ids.size(); i++)
            {
                if ((fabs(surf_residuals[2 * i + 0]) + fabs(surf_residuals[2 * i + 1])) > m_inlier_threshold_points) // std::min( 1.0, 10 * avr_cost )
                {
                    problem.RemoveResidualBlock(mappoints_residual_block_ids[i]);
                }
                else
                {
                    mappoints_residual_block_ids_temp.push_back(mappoints_residual_block_ids[i]);
                }
            }
        #endif
        
            corner_residual_block_ids = corner_residual_block_ids_temp;
            surf_residual_block_ids = surf_residual_block_ids_temp;
        
        #ifdef USE_VISUAL_FEATURES
            mappoints_residual_block_ids = mappoints_residual_block_ids_temp;
        #endif

            //重新进行优化
            options.linear_solver_type                = slover_type;
            options.max_num_iterations                = m_para_cere_max_iterations;
            options.minimizer_progress_to_stdout      = false;
            options.check_gradients                   = false;
            options.gradient_check_relative_precision = 1e-10;

            // set_ceres_solver_bound( problem, m_para_buffer_incremental );
            ceres::Solve(options, &problem, &summary);
        }

        //更新当前帧的位姿
        Eigen::Matrix4d tcw   = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d r_cw  = m_q_w_curr.toRotationMatrix().transpose();
        tcw.block<3, 3>(0, 0) =  r_cw;
        tcw.block<3, 1>(0, 3) = -r_cw * m_t_w_curr;
        pFrame->SetPose(Converter::toCvMat(tcw));

        //更新mapPoints信息
        // int nInlier = 0;
        // for(int i=0;i<pFrame->N;i++)
        // {
        //     MapPoint* pMP = pFrame->mvpMapPoints[i];
        //     if(!pMP)
        //         continue;
                
        //     Eigen::Vector3d pw = Converter::toVector3d(pMP->GetWorldPos());
        //     Eigen::Vector3d pc = tcw.block<3,3>(0,0) * pw + tcw.block<3,1>(0,3);
        //     Eigen::Vector2d uv = pFrame->mpPinholeCamera->world2cam(pc);
        //     cv::KeyPoint kpUn  = pFrame->mvKeysUn[i];
        //     const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
        //     Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;
        //     Eigen::Vector2d residual = Eigen::Vector2d(kpUn.pt.x,kpUn.pt.y) - uv;
        //     float chi2 = residual.transpose() * information * residual;
        //     if(chi2 > sqrt(5.991))
        //          pFrame->mvbOutlier[i] = true;
        //     else
        //         nInlier++;
        // }
        // std::cout<<"nInlier: "<<nInlier<<std::endl;
    }
}


float CeresOptimizer::PoseOptimization(Eigen::Matrix4d& transform_wc,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalCornerMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pLocalSurfMap,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurCornerScan,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &pCurSurfScan,
    int max_iteration)
{
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0}; //orentation translation
    Eigen::Map<Eigen::Quaterniond> m_q_w_curr(parameters);
    Eigen::Map<Eigen::Vector3d> m_t_w_curr(parameters + 4);

    Eigen::Matrix4d Twc = transform_wc;
    Eigen::Quaterniond q(Twc.topLeftCorner<3, 3>(0, 0));
    q.normalize();
    Eigen::Vector3d t = Twc.block<3, 1>(0, 3);

    //6DOF优化初值
    parameters[0] = q.x();
    parameters[1] = q.y();
    parameters[2] = q.z();
    parameters[3] = q.w();
    parameters[4] = t[0];
    parameters[5] = t[1];
    parameters[6] = t[2];

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    PointType pointOri, pointSel, pointProj, coeff;

    const int laserCloudCornerLastDSNum = pCurCornerScan->size();
    const int laserCloudSurfLastDSNum   = pCurSurfScan->size();

    ceres::LinearSolverType slover_type = ceres::DENSE_SCHUR; // SPARSE_NORMAL_CHOLESKY | DENSE_QR | DENSE_SCHUR
    int m_para_cere_max_iterations      = 100;
    int m_para_cere_prerun_times        = 2;

    double m_inlier_ratio       = 0.80;
    double m_inliner_dis        = 0.02;
    double m_inliner_dis_visual = 5.0;
    double m_inlier_threshold_corner;
    double m_inlier_threshold_surf;
    double m_inlier_threshold_points;

    float final_cost = 999999999.0;

    // 6-DOF 优化
    if (pLocalCornerMap->size() > 10 && pLocalSurfMap->size() > 10)
    {
        pcl::KdTreeFLANN<PointType> m_kdtree_corner_from_map;
        pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map;
        m_kdtree_corner_from_map.setInputCloud(pLocalCornerMap);
        m_kdtree_surf_from_map.setInputCloud(pLocalSurfMap);

        for (int iterCount = 0; iterCount < max_iteration; iterCount++)
        {
            ceres::LossFunction *loss_function               = new ceres::HuberLoss(0.1);
            ceres::LossFunction *visual_loss_function        = new ceres::HuberLoss(sqrt(5.991));
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::ResidualBlockId block_id;
            ceres::Problem problem(problem_options);
            std::vector<ceres::ResidualBlockId> mappoints_residual_block_ids;
            std::vector<ceres::ResidualBlockId> corner_residual_block_ids;
            std::vector<ceres::ResidualBlockId> surf_residual_block_ids;

            problem.AddParameterBlock(parameters, 4, q_parameterization);
            problem.AddParameterBlock(parameters + 4, 3);
               
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Add Corner Points
            int corner_num = 0;
            int corner_num_rejected = 0;
            for (int i = 0; i < laserCloudCornerLastDSNum; i++)
            {
                pointOri = pCurCornerScan->points[i];
                pointAssociateToMap(&pointOri, m_q_w_curr, m_t_w_curr, &pointSel);
                m_kdtree_corner_from_map.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                if (pointSearchSqDis[4] < 1.0)
                {
                    std::vector<Eigen::Vector3d> nearCorners;
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Vector3d tmp(pLocalCornerMap->points[pointSearchInd[j]].x,
                                            pLocalCornerMap->points[pointSearchInd[j]].y,
                                            pLocalCornerMap->points[pointSearchInd[j]].z);
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

                    // if is indeed line feature
                    // note Eigen library sort eigenvalues in increasing order
                    Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    {
                        Eigen::Vector3d point_on_line = center;
                        Eigen::Vector3d point_a, point_b;
                        point_a =  0.1 * unit_direction + point_on_line;
                        point_b = -0.1 * unit_direction + point_on_line;

                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                        block_id = problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                        corner_residual_block_ids.push_back(block_id);
                        corner_num++;
                    }
                    else
                    {
                        corner_num_rejected++;
                    }
                }
            }

            // Add Surf Points
            int surf_num = 0;
            int surf_rejected_num = 0;
            for (int i = 0; i < laserCloudSurfLastDSNum; i++)
            {
                pointOri = pCurSurfScan->points[i];
                pointAssociateToMap(&pointOri, m_q_w_curr, m_t_w_curr, &pointSel);
                m_kdtree_surf_from_map.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < 1.0)
                {

                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = pLocalSurfMap->points[pointSearchInd[j]].x;
                        matA0(j, 1) = pLocalSurfMap->points[pointSearchInd[j]].y;
                        matA0(j, 2) = pLocalSurfMap->points[pointSearchInd[j]].z;
                        //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                    }
                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * pLocalSurfMap->points[pointSearchInd[j]].x +
                                 norm(1) * pLocalSurfMap->points[pointSearchInd[j]].y +
                                 norm(2) * pLocalSurfMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    if (planeValid)
                    {
                        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                        block_id = problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                        surf_residual_block_ids.push_back(block_id);
                        surf_num++;
                    }
                    else
                    {
                        surf_rejected_num++;
                    }
                }
            }

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double t_time= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            std::cout<<"time of associate features: "<<t_time<<std::endl;

            //优化
            ceres::Solver::Options options;
            options.linear_solver_type = /*ceres::DENSE_QR*/ slover_type;
            options.max_num_iterations = /*100*/ m_para_cere_prerun_times;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-6;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // 计算边缘的残差,剔除错误的匹配
            ceres::Problem::EvaluateOptions eval_options;
            eval_options.residual_blocks = corner_residual_block_ids;
            double corner_total_cost = 0.0;
            std::vector<double> corner_residuals; //每个观测的误差
            problem.Evaluate(eval_options, &corner_total_cost, &corner_residuals, nullptr, nullptr);

            // 计算平坦点的残差,剔除错误的匹配
            eval_options.residual_blocks = surf_residual_block_ids;
            double surf_total_cost = 0.0;
            std::vector<double> surf_residuals; //每个观测的误差
            problem.Evaluate(eval_options, &surf_total_cost, &surf_residuals, nullptr, nullptr);

            //边缘点
            std::vector<ceres::ResidualBlockId> corner_residual_block_ids_temp;
            corner_residual_block_ids_temp.reserve(corner_residual_block_ids.size());
            double m_inliner_ratio_threshold_corner = compute_inlier_residual_threshold_corner(corner_residuals, m_inlier_ratio);
            m_inlier_threshold_corner = std::max(m_inliner_dis, m_inliner_ratio_threshold_corner);

            //平坦点
            std::vector<ceres::ResidualBlockId> surf_residual_block_ids_temp;
            surf_residual_block_ids_temp.reserve(surf_residual_block_ids.size());
            double m_inliner_ratio_threshold_surf = compute_inlier_residual_threshold_surf(surf_residuals, m_inlier_ratio);
            m_inlier_threshold_surf = std::max(m_inliner_dis, m_inliner_ratio_threshold_surf);


            //剔除误差较大的边缘激光点
            for (unsigned int i = 0; i < corner_residual_block_ids.size(); i++)
            {
                if ((fabs(corner_residuals[3 * i + 0]) + fabs(corner_residuals[3 * i + 1]) + fabs(corner_residuals[3 * i + 2])) > m_inlier_threshold_corner) // std::min( 1.0, 10 * avr_cost )
                {
                    //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                    problem.RemoveResidualBlock(corner_residual_block_ids[i]);
                }
                else
                {
                    corner_residual_block_ids_temp.push_back(corner_residual_block_ids[i]);
                }
            }

            //剔除误差较大的平坦点激光点
            for (unsigned int i = 0; i < surf_residual_block_ids.size(); i++)
            {
                if (fabs(surf_residuals[i]) > m_inlier_threshold_surf) // std::min( 1.0, 10 * avr_cost )
                {
                    //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                    problem.RemoveResidualBlock(surf_residual_block_ids[i]);
                }
                else
                {
                    surf_residual_block_ids_temp.push_back(surf_residual_block_ids[i]);
                }
            }
       
            corner_residual_block_ids = corner_residual_block_ids_temp;
            surf_residual_block_ids = surf_residual_block_ids_temp;
        
            //重新进行优化
            options.linear_solver_type                = slover_type;
            options.max_num_iterations                = m_para_cere_max_iterations;
            options.minimizer_progress_to_stdout      = false;
            options.check_gradients                   = false;
            options.gradient_check_relative_precision = 1e-10;

            // set_ceres_solver_bound( problem, m_para_buffer_incremental );
            ceres::Solve(options, &problem, &summary);
            final_cost = summary.final_cost;
        }

        //更新当前帧的位姿
        transform_wc.block<3, 3>(0, 0) = m_q_w_curr.toRotationMatrix();
        transform_wc.block<3, 1>(0, 3) = m_t_w_curr;
    }

    return final_cost;
}





//3d-2d位置优化
int CeresOptimizer::Frame2FramePoseOptimization(Frame* pFrame)
{
    //orentation translation
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};          
    Eigen::Map<Eigen::Quaterniond> m_q_curr_w(parameters);
    Eigen::Map<Eigen::Vector3d> m_t_curr_w(parameters + 4);

    Eigen::Matrix4d Twc = Converter::toMatrix4d(pFrame->mTcw).inverse();
    Eigen::Quaterniond q(Twc.topLeftCorner<3, 3>(0, 0));
    q.normalize();
    Eigen::Vector3d t = Twc.block<3, 1>(0, 3);

    //6DOF优化初值
    parameters[0] = q.x();
    parameters[1] = q.y();
    parameters[2] = q.z();
    parameters[3] = q.w();
    parameters[4] = t[0];
    parameters[5] = t[1];
    parameters[6] = t[2];

    ceres::LossFunction *visual_loss_function        = new ceres::HuberLoss(sqrt(5.991));
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;
    ceres::ResidualBlockId block_id;
    ceres::Problem problem(problem_options);
    std::vector<ceres::ResidualBlockId> mappoints_residual_block_ids;
    problem.AddParameterBlock(parameters, 4, q_parameterization);
    problem.AddParameterBlock(parameters + 4, 3);

    int visual_num = 0;
    for (int i=0;i<pFrame->N;i++)
    {
        MapPoint *pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // if(pFrame->mvbOutlier[i])
            //     continue;
            Eigen::Vector3d  pos_w  = Converter::toVector3d(pMP->GetWorldPos());
            
            // Monocular observation
            cv::KeyPoint kpUn = pFrame->mvKeysUn[i];        

            ceres::CostFunction *cost_function = SnavelyReprojectionFactorPoseOnly::Create(pos_w,kpUn.pt.x, kpUn.pt.y,pFrame->mpPinholeCamera);
            block_id = problem.AddResidualBlock(cost_function, visual_loss_function, parameters, parameters + 4);
            mappoints_residual_block_ids.push_back(block_id);

            visual_num++;
        }
    }
    std::cout<<"Add visual mappoints: "<<visual_num<<std::endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-6;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Matrix4d tcw = Eigen::Matrix4d::Identity();
    tcw.block<3, 3>(0, 0) = m_q_curr_w.toRotationMatrix();
    tcw.block<3, 1>(0, 3) = m_t_curr_w;
    // pFrame->SetPose(Converter::toCvMat(tcw));

    std::cout<<"ceres optimization : "<<std::endl<<tcw.matrix()<<std::endl;
}








int CeresOptimizer::PoseOptimization(KeyFrame *pKF, Map *pMap)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame *> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    const int N_FRAMES = lFixedCameras.size() + lLocalKeyFrames.size();
    // std::cout << "Local KeyFrame size: " << lFixedCameras.size() + lLocalKeyFrames.size() << std::endl;

    //(1)提取当前关键帧附近的激光点云地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr pCornerMap(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pSurfMap(new pcl::PointCloud<pcl::PointXYZI>());
    for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        pcl::PointCloud<PointType>::Ptr transform_pointcloud(new pcl::PointCloud<PointType>());
        Eigen::Matrix4d Twc = Converter::toMatrix4d((*lit)->GetPoseInverse()).matrix();
        // Eigen::Matrix4d transform = Twc * pMap->lidar_to_camera_;
        Eigen::Matrix4d transform = Twc;

        pcl::transformPointCloud(
            *((*lit)->mpCornerPointsLessSharp),
            *transform_pointcloud,
            transform);

        *pCornerMap += *transform_pointcloud;

        pcl::transformPointCloud(
            *((*lit)->mpSurfPointsLessFlat),
            *transform_pointcloud,
            transform);

        *pSurfMap += *transform_pointcloud;
    }

    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        pcl::PointCloud<PointType>::Ptr transform_pointcloud(new pcl::PointCloud<PointType>());
        Eigen::Matrix4d Twc = Converter::toMatrix4d((*lit)->GetPoseInverse()).matrix();
        // Eigen::Matrix4d transform = Twc * pMap->lidar_to_camera_;
        Eigen::Matrix4d transform = Twc;

        pcl::transformPointCloud(
            *((*lit)->mpCornerPointsLessSharp),
            *transform_pointcloud,
            transform);

        *pCornerMap += *transform_pointcloud;

        pcl::transformPointCloud(
            *((*lit)->mpSurfPointsLessFlat),
            *transform_pointcloud,
            transform);

        *pSurfMap += *transform_pointcloud;
    }

    //(2)对当前帧激光雷达进行降采样
    //2.1 将激光雷达转换到相机坐标系中并进行降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_corner_ds(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_surf_ds(new pcl::PointCloud<pcl::PointXYZI>());

    // pcl::PointCloud<PointType>::Ptr tmp_pointcloud(new pcl::PointCloud<PointType>());
    // pcl::transformPointCloud(
    //     *(pKF->mpCornerPointsLessSharp),
    //     *tmp_pointcloud,
    //     pMap->lidar_to_camera_);
    pMap->mDownSizeFilterCorner.setInputCloud(pKF->mpCornerPointsLessSharp);
    pMap->mDownSizeFilterCorner.filter(*cur_corner_ds);

    // pcl::transformPointCloud(
    //     *(pKF->mpSurfPointsLessFlat),
    //     *tmp_pointcloud,
    //     pMap->lidar_to_camera_);
    pMap->mDownSizeFilterSurf.setInputCloud(pKF->mpSurfPointsLessFlat);
    pMap->mDownSizeFilterSurf.filter(*cur_surf_ds);

    //(3)构造ceres优化器进行优化
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0}; //orentation translation
    Eigen::Map<Eigen::Quaterniond> m_q_w_curr(parameters);
    Eigen::Map<Eigen::Vector3d> m_t_w_curr(parameters + 4);

    Eigen::Matrix4d Twc = Converter::toMatrix4d(pKF->GetPoseInverse()).matrix();
    Eigen::Quaterniond q(Twc.topLeftCorner<3, 3>(0, 0));
    q.normalize();
    Eigen::Vector3d t = Twc.block<3, 1>(0, 3);

    //6DOF优化初值
    parameters[0] = q.x();
    parameters[1] = q.y();
    parameters[2] = q.z();
    parameters[3] = q.w();
    parameters[4] = t[0];
    parameters[5] = t[1];
    parameters[6] = t[2];

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    PointType pointOri, pointSel, pointProj, coeff;

    const int laserCloudCornerLastDSNum = cur_corner_ds->size();
    const int laserCloudSurfLastDSNum   = cur_surf_ds->size();

    ceres::LinearSolverType slover_type = ceres::DENSE_SCHUR; // SPARSE_NORMAL_CHOLESKY | DENSE_QR | DENSE_SCHUR
    int m_para_cere_max_iterations      = 100;
    int m_para_cere_prerun_times        = 2;

    double m_inlier_ratio = 0.80;
    double m_inliner_dis = 0.02;
    double m_inlier_threshold_corner;
    double m_inlier_threshold_surf;

    // 6-DOF 优化
    if (pCornerMap->size() > 10 && pSurfMap->size() > 10)
    {
        pcl::KdTreeFLANN<PointType> m_kdtree_corner_from_map;
        pcl::KdTreeFLANN<PointType> m_kdtree_surf_from_map;
        m_kdtree_corner_from_map.setInputCloud(pCornerMap);
        m_kdtree_surf_from_map.setInputCloud(pSurfMap);

        int iteration = 3;
        // if (N_FRAMES < 10)
        //     iteration = 5;
        // else
        //     iteration = 3;

        for (int iterCount = 0; iterCount < iteration; iterCount++)
        {
            ceres::LossFunction *loss_function               = new ceres::HuberLoss(0.1);
            ceres::LossFunction *visual_loss_function        = new ceres::HuberLoss(sqrt(5.991));
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::ResidualBlockId block_id;
            ceres::Problem problem(problem_options);
            std::vector<ceres::ResidualBlockId> mappoints_residual_block_ids;
            std::vector<ceres::ResidualBlockId> corner_residual_block_ids;
            std::vector<ceres::ResidualBlockId> surf_residual_block_ids;

            problem.AddParameterBlock(parameters, 4, q_parameterization);
            problem.AddParameterBlock(parameters + 4, 3);

            //Add MapPoints
            // int visual_num = 0;
            // std::vector<Eigen::Vector3d> mappoints_pos_w;
            // for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
            // {
            //     MapPoint *pMP = *lit;
                
            //     const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            //     Eigen::Vector3d  pos_w = Converter::toVector3d(pMP->GetWorldPos());
            //     mappoints_pos_w.push_back(pos_w);

            //     //Set edges
            //     for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            //     {
            //         KeyFrame *pKFi = mit->first;

            //         if (!pKFi->isBad())
            //         {
            //             const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

            //             // Monocular observation
            //             Eigen::Matrix<double, 2, 1> obs;
            //             obs << kpUn.pt.x, kpUn.pt.y;
                        
            //             ceres::CostFunction *cost_function = SnavelyReprojectionFactorPoseOnly::Create(pos_w,obs.x(),obs.y(),pKFi->mpPinholeCamera);
            //             block_id = problem.AddResidualBlock(cost_function, visual_loss_function, parameters, parameters + 4);
            //             mappoints_residual_block_ids.push_back(block_id);
                        
            //         }
            //     }
            // }

            // Add Corner Points
            int corner_num = 0;
            int corner_num_rejected = 0;
            for (int i = 0; i < laserCloudCornerLastDSNum; i++)
            {
                pointOri = cur_corner_ds->points[i];
                pointAssociateToMap(&pointOri, m_q_w_curr, m_t_w_curr, &pointSel);
                m_kdtree_corner_from_map.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                if (pointSearchSqDis[4] < 1.0)
                {
                    std::vector<Eigen::Vector3d> nearCorners;
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Vector3d tmp(pCornerMap->points[pointSearchInd[j]].x,
                                            pCornerMap->points[pointSearchInd[j]].y,
                                            pCornerMap->points[pointSearchInd[j]].z);
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

                    // if is indeed line feature
                    // note Eigen library sort eigenvalues in increasing order
                    Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    {
                        Eigen::Vector3d point_on_line = center;
                        Eigen::Vector3d point_a, point_b;
                        point_a = 0.1 * unit_direction + point_on_line;
                        point_b = -0.1 * unit_direction + point_on_line;

                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                        block_id = problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                        corner_residual_block_ids.push_back(block_id);
                        corner_num++;
                    }
                    else
                    {
                        corner_num_rejected++;
                    }
                }
            }

            // Add Surf Points
            int surf_num = 0;
            int surf_rejected_num = 0;
            for (int i = 0; i < laserCloudSurfLastDSNum; i++)
            {
                pointOri = cur_surf_ds->points[i];
                pointAssociateToMap(&pointOri, m_q_w_curr, m_t_w_curr, &pointSel);
                //double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
                m_kdtree_surf_from_map.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < 1.0)
                {

                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = pSurfMap->points[pointSearchInd[j]].x;
                        matA0(j, 1) = pSurfMap->points[pointSearchInd[j]].y;
                        matA0(j, 2) = pSurfMap->points[pointSearchInd[j]].z;
                        //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                    }
                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * pSurfMap->points[pointSearchInd[j]].x +
                                 norm(1) * pSurfMap->points[pointSearchInd[j]].y +
                                 norm(2) * pSurfMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                    if (planeValid)
                    {
                        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                        block_id = problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                        surf_residual_block_ids.push_back(block_id);
                        surf_num++;
                    }
                    else
                    {
                        surf_rejected_num++;
                    }
                }
            }

            //优化
            ceres::Solver::Options options;
            options.linear_solver_type = /*ceres::DENSE_QR*/ slover_type;
            options.max_num_iterations = /*100*/ m_para_cere_prerun_times;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-6;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // 计算边缘的残差,剔除错误的匹配
            ceres::Problem::EvaluateOptions eval_options;
            eval_options.residual_blocks = corner_residual_block_ids;
            double corner_total_cost = 0.0;
            std::vector<double> corner_residuals; //每个观测的误差
            problem.Evaluate(eval_options, &corner_total_cost, &corner_residuals, nullptr, nullptr);

            // 计算平坦点的残差,剔除错误的匹配
            eval_options.residual_blocks = surf_residual_block_ids;
            double surf_total_cost = 0.0;
            std::vector<double> surf_residuals; //每个观测的误差
            problem.Evaluate(eval_options, &surf_total_cost, &surf_residuals, nullptr, nullptr);

            //
            std::vector<ceres::ResidualBlockId> corner_residual_block_ids_temp;
            corner_residual_block_ids_temp.reserve(corner_residual_block_ids.size());
            double m_inliner_ratio_threshold_corner = compute_inlier_residual_threshold_corner(corner_residuals, m_inlier_ratio);
            m_inlier_threshold_corner = std::max(m_inliner_dis, m_inliner_ratio_threshold_corner);

            std::vector<ceres::ResidualBlockId> surf_residual_block_ids_temp;
            surf_residual_block_ids_temp.reserve(surf_residual_block_ids.size());
            double m_inliner_ratio_threshold_surf = compute_inlier_residual_threshold_surf(surf_residuals, m_inlier_ratio);
            m_inlier_threshold_surf = std::max(m_inliner_dis, m_inliner_ratio_threshold_surf);

            //剔除误差较大的边缘激光点
            for (unsigned int i = 0; i < corner_residual_block_ids.size(); i++)
            {
                if ((fabs(corner_residuals[3 * i + 0]) + fabs(corner_residuals[3 * i + 1]) + fabs(corner_residuals[3 * i + 2])) > m_inlier_threshold_corner) // std::min( 1.0, 10 * avr_cost )
                {
                    //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                    problem.RemoveResidualBlock(corner_residual_block_ids[i]);
                }
                else
                {
                    corner_residual_block_ids_temp.push_back(corner_residual_block_ids[i]);
                }
            }

            //剔除误差较大的平坦点激光点
            for (unsigned int i = 0; i < surf_residual_block_ids.size(); i++)
            {
                if ((fabs(surf_residuals[3 * i + 0]) + fabs(surf_residuals[3 * i + 1]) + fabs(surf_residuals[3 * i + 2])) > m_inlier_threshold_surf) // std::min( 1.0, 10 * avr_cost )
                {
                    //screen_out << "Remove outliers, drop id = " << (void *)residual_block_ids[ i ] <<endl;
                    problem.RemoveResidualBlock(surf_residual_block_ids[i]);
                }
                else
                {
                    surf_residual_block_ids_temp.push_back(surf_residual_block_ids[i]);
                }
            }

            corner_residual_block_ids = corner_residual_block_ids_temp;
            surf_residual_block_ids = surf_residual_block_ids_temp;

            //重新进行优化
            options.linear_solver_type = slover_type;
            options.max_num_iterations = m_para_cere_max_iterations;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-10;

            // set_ceres_solver_bound( problem, m_para_buffer_incremental );
            ceres::Solve(options, &problem, &summary);
        }

        //更新当前帧的位姿
        Eigen::Matrix4d tcw = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d r_cw = m_q_w_curr.toRotationMatrix().transpose();
        tcw.block<3, 3>(0, 0) = r_cw;
        tcw.block<3, 1>(0, 3) = -r_cw * m_t_w_curr;
        pKF->SetPose(Converter::toCvMat(tcw));
        
        //将当前帧加入到局部地图中
    }
}





int CeresOptimizer::CurveFitting()
{
    double a = 1.0, b = 2.0, c = 1.0; // 真实参数值
    int N = 100;                      // 数据点
    double w_sigma = 1.0;             // 噪声Sigma值
    cv::RNG rng;                      // OpenCV随机数产生器
    double abc[3] = {0, 0, 0};        // abc参数的估计值

    std::vector<double> x_data, y_data; // 数据

    std::cout << "generating data: " << std::endl;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(
            exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        // cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    // 构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        ceres::CostFunction *cost_function = CurveFittingCostFunctor::Create(x_data[i], y_data[i]);
        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.), abc);
    }

    // 配置求解器
    ceres::Solver::Options options;               // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;  // 输出到cout

    ceres::Solver::Summary summary; // 优化信息
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary); // 开始优化
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    // 输出结果
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated a,b,c = ";
    for (auto a : abc)
        std::cout << a << " ";
    std::cout << std::endl;
}

} // namespace ORB_SLAM2