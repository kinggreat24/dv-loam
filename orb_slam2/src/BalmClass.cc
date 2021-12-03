/*
 * @Author: kinggreat24
 * @Date: 2021-04-08 10:10:41
 * @LastEditTime: 2021-04-08 11:16:06
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/src/BalmClass.cc
 * 可以输入预定的版权声明、个性签名、空行等
 */

#include "BalmClass.h"

namespace ORB_SLAM2{

Points_cloud_cell::Points_cloud_cell()
{
    m_mutex_cell = std::make_shared<std::mutex>();
    clear_data();
}

Points_cloud_cell::Points_cloud_cell(const PT_TYPE &cell_center, const DATA_TYPE &res)
{
    m_mutex_cell = std::make_shared<std::mutex>();
    clear_data();
    m_resolution = res;
    m_maximum_points_size = (int)res * 100.0;
    m_points_vec.reserve(m_maximum_points_size);
    m_center = cell_center;
    //append_pt( cell_center );
}

Points_cloud_cell::~Points_cloud_cell()
{
    m_mutex_cell->try_lock();
    m_mutex_cell->unlock();
    clear_data();
}

void Points_cloud_cell::set_data_need_update(int if_update_sum)
{
    m_mean_need_update = true;
    m_covmat_need_update = true;
    m_icovmat_need_update = true;
    m_last_eigen_decompose_size = 0;
    if (if_update_sum)
    {
        m_xyz_sum.setZero();
        for (size_t i = 0; i < m_points_vec.size(); i++)
        {
            m_xyz_sum += m_points_vec[i].template cast<COMP_TYPE>();
        }
    }
}

Eigen::Matrix<DATA_TYPE, 3, 1> Points_cloud_cell::get_mean()
{
    //std::unique_lock<std::mutex> lock(*m_mutex_cell_mean);
    if (m_if_incremental_update_mean_and_cov == false)
    {
        if (m_mean_need_update)
        {
            set_data_need_update();
            m_mean = m_xyz_sum / ((DATA_TYPE)(m_points_vec.size()));
        }
        m_mean_need_update = false;
    }
    return m_mean.template cast<DATA_TYPE>();
}

void Points_cloud_cell::covmat_eig_decompose(int if_force_update)
{
    if (if_force_update == 0) //if update is not so large, skip eigendecompse to save time.
    {
        if (m_last_eigen_decompose_size > 1000 && (m_last_eigen_decompose_size / m_points_vec.size() > 0.90))
        {
            return;
        }
    }
    m_last_eigen_decompose_size = m_points_vec.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<COMP_TYPE, 3, 3>> eigensolver;
    eigensolver.compute(m_cov_mat);
    m_eigen_val = eigensolver.eigenvalues();
    m_eigen_vec = eigensolver.eigenvectors();
}

Eigen::Matrix<COMP_TYPE, 3, 3> Points_cloud_cell::get_cov_mat_avoid_singularity() // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]
{
    covmat_eig_decompose();
    Eigen::Matrix<COMP_TYPE, 3, 3> eigen_val = m_eigen_val.asDiagonal();
    Eigen::Matrix<COMP_TYPE, 3, 3> res_cov_mat;
    COMP_TYPE min_covar_eigvalue;
    COMP_TYPE min_covar_eigvalue_mult_ = 0.01; // pcl: 0.01
    if (!IF_EIGEN_REPLACE)
        min_covar_eigvalue_mult_ = 0;
    min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val(2, 2);
    if (eigen_val(0, 0) < min_covar_eigvalue)
    {
        eigen_val(0, 0) = min_covar_eigvalue;

        if (eigen_val(1, 1) < min_covar_eigvalue)
        {
            eigen_val(1, 1) = min_covar_eigvalue;
        }

        res_cov_mat = m_eigen_vec * eigen_val * m_eigen_vec.inverse();
        if (!std::isfinite(m_cov_mat(0, 0)))
        {
            res_cov_mat.setIdentity();
        }
    }

    return res_cov_mat;
}

Eigen::Matrix<DATA_TYPE, 3, 3> Points_cloud_cell::get_covmat()
{
    // std::unique_lock<std::mutex> lock(*m_mutex_cell_cov);
    if (m_covmat_need_update)
    {
        get_mean();
        size_t pt_size = m_points_vec.size();
        if (pt_size < 5 || m_if_incremental_update_mean_and_cov == false)
        {
            if (IF_COV_INIT_IDENTITY)
            {
                m_cov_mat.setIdentity();
            }
            else
            {
                m_cov_mat.setZero();
            }

            if (pt_size <= 2)
            {
                return m_cov_mat.template cast<DATA_TYPE>();
            }

            for (size_t i = 0; i < pt_size; i++)
            {
                m_cov_mat = m_cov_mat + (m_points_vec[i] * m_points_vec[i].transpose()).template cast<COMP_TYPE>();
            }

            m_cov_mat -= pt_size * (m_mean * m_mean.transpose());
            m_cov_mat /= (pt_size - 1);
        }
        m_cov_mat_avoid_singularity = get_cov_mat_avoid_singularity();
    }
    m_covmat_need_update = false;
    return m_cov_mat_avoid_singularity.template cast<DATA_TYPE>();
}

Eigen::Matrix<DATA_TYPE, 3, 3> Points_cloud_cell::get_icovmat()
{
    if (m_icovmat_need_update)
    {
        get_covmat();
        m_icov_mat = m_cov_mat.inverse();
        if (!std::isfinite(m_icov_mat(0, 0)))
        {
            m_icov_mat.setIdentity();
        }
    }
    m_icovmat_need_update = false;
    return m_icov_mat.template cast<DATA_TYPE>();
}

pcl::PointCloud<pcl_pt> Points_cloud_cell::get_pointcloud()
{
    std::unique_lock<std::mutex> lock(*m_mutex_cell);
    if (!IF_ENABLE_DUMP_PCL_PTS)
    {
        m_pcl_pc_vec = PCL_TOOLS::eigen_pt_to_pcl_pointcloud<pcl_pt, Eigen_Point>(m_points_vec);
    }
    return m_pcl_pc_vec;
}

std::vector<PT_TYPE> Points_cloud_cell::get_pointcloud_eigen()
{
    std::unique_lock<std::mutex> lock(*m_mutex_cell);
    return m_points_vec;
}

void Points_cloud_cell::set_pointcloud(pcl::PointCloud<pcl_pt> &pc_in)
{
    std::unique_lock<std::mutex> lock(*m_mutex_cell);
    m_pcl_pc_vec = pc_in;
    m_points_vec = PCL_TOOLS::pcl_pts_to_eigen_pts<float, pcl_pt>(pc_in.makeShared());
}

void Points_cloud_cell::clear_data()
{
    std::unique_lock<std::mutex> lock(*m_mutex_cell);
    m_points_vec.clear();
    std::vector<PT_TYPE>().swap(m_points_vec);
    m_pcl_pc_vec.clear();
    m_mean.setZero();
    m_xyz_sum.setZero();
    m_cov_mat.setZero();
    set_data_need_update();
}

	
void Points_cloud_cell::append_pt(const PT_TYPE &pt)
{
    std::unique_lock<std::mutex> lock(*m_mutex_cell);
    if (IF_ENABLE_DUMP_PCL_PTS)
    {
        m_pcl_pc_vec.push_back(PCL_TOOLS::eigen_to_pcl_pt<pcl_pt>(pt));
    }
    m_points_vec.push_back(pt);
    if (m_points_vec.size() > m_maximum_points_size)
    {
        m_maximum_points_size *= 2;
        m_points_vec.reserve(m_maximum_points_size);
    }

    m_xyz_sum = m_xyz_sum + pt.template cast<COMP_TYPE>();

    if (m_if_incremental_update_mean_and_cov)
    {
        m_mean_last = m_mean;
        m_cov_mat_last = m_cov_mat;

        auto P_new = pt.template cast<COMP_TYPE>();
        size_t N = m_points_vec.size() - 1;
        m_mean = (N * m_mean_last + P_new) / (N + 1);

        if (N > 5)
        {
            m_cov_mat = ((N - 1) * m_cov_mat_last + (P_new - m_mean_last) * ((P_new - m_mean_last).transpose()) +
                            (N + 1) * (m_mean_last - m_mean) * ((m_mean_last - m_mean).transpose()) +
                            2 * (m_mean_last - m_mean) * ((P_new - m_mean_last).transpose())) /
                        N;
        }
        else
        {
            get_covmat();
        }
    }
    m_last_update_time = ORB_SLAM2::timer_tic();
    set_data_need_update();
    //m_points_vec.insert(pt);
}

void Points_cloud_cell::set_target_pc(const std::vector<PT_TYPE> &pt_vec)
{
    std::unique_lock<std::mutex> lock(*m_mutex_cell);
    // "The three-dimensional normal-distributions transform: an efficient representation for registration, surface analysis, and loop detection"
    // Formulation 6.2 and 6.3
    int pt_size = pt_vec.size();
    clear_data();
    m_points_vec.reserve(m_maximum_points_size);
    for (int i = 0; i < pt_size; i++)
    {
        append_pt(pt_vec[i]);
    }
    set_data_need_update();
}

Feature_type Points_cloud_cell::determine_feature(int if_recompute)
{
    std::unique_lock<std::mutex> lock(*m_mutex_cell);
    if (if_recompute)
    {
        set_data_need_update(1);
    }

    m_feature_type = e_feature_sphere;
    if (m_points_vec.size() < 5)
    {
        m_feature_type = e_feature_sphere;
        m_feature_vector << 0, 0, 0;
        return e_feature_sphere;
    }

    get_covmat();

    if ((m_center.template cast<float>() - m_mean.template cast<float>()).norm() > m_resolution * 0.75)
    {
        m_feature_type = e_feature_sphere;
        m_feature_vector << 0, 0, 0;
        return e_feature_sphere;
    }

    if ((m_eigen_val[1] * m_feature_determine_threshold_plane > m_eigen_val[0]))
    {
        m_feature_type = e_feature_plane;
        m_feature_vector = m_eigen_vec.block<3, 1>(0, 0);
        return m_feature_type;
    }
    if (m_eigen_val[2] * m_feature_determine_threshold_line > m_eigen_val[1])
    {
        m_feature_type = e_feature_line;
        m_feature_vector = m_eigen_vec.block<3, 1>(0, 2);
    }
    return m_feature_type;
}
   
}