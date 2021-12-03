/*
 * @Author: kinggreat24
 * @Date: 2021-04-01 16:23:20
 * @LastEditTime: 2021-04-08 20:18:58
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/orb_slam2/include/BalmClass.h
 * 可以输入预定的版权声明、个性签名、空行等
 */

#ifndef BALM_CLASS_H
#define BALM_CALSS_H

#include <map>
#include <hashtable.h>
#include <unordered_map>
#include <eigen3/Eigen/Core>
#include "Common.h"
#include "pcl_tools.hpp"
#include <mutex>
#include <memory>

typedef float COMP_TYPE;
typedef float DATA_TYPE;
typedef pcl::PointXYZI pcl_pt;
typedef pcl::PointXYZRGB pcl_rgb_pt;
typedef Eigen::Matrix<DATA_TYPE, 3, 1> Eigen_Point;
typedef Eigen::Matrix<DATA_TYPE, 3, 1> PT_TYPE;

#define IF_COV_INIT_IDENTITY 0
#define IF_EIGEN_REPLACE 1
#define IF_ENABLE_INCREMENTAL_UPDATE_MEAN_COV 0
#define IF_ENABLE_DUMP_PCL_PTS 0

namespace ORB_SLAM2
{

	static std::chrono::time_point<std::chrono::system_clock> timer_now()
	{
		return std::chrono::system_clock::now();
	}

	static double timer_tic()
	{
		//std::time_t t = std::chrono::system_clock::to_time_t( timer_now() );
		auto t = std::chrono::system_clock::to_time_t(timer_now());
		return double(t);
	}

	enum Feature_type
	{
		e_feature_sphere = 0,
		e_feature_line = 1,
		e_feature_plane = 2
	};

	// 点云格网
	class Points_cloud_cell
	{
	public:
		DATA_TYPE m_resolution;
		Eigen::Matrix<DATA_TYPE, 3, 1> m_center;

		//private:
		Eigen::Matrix<COMP_TYPE, 3, 1> m_xyz_sum;
		Eigen::Matrix<COMP_TYPE, 3, 1> m_mean, m_mean_last;
		Eigen::Matrix<COMP_TYPE, 3, 3> m_cov_mat, m_cov_mat_last, m_cov_mat_avoid_singularity;
		Eigen::Matrix<COMP_TYPE, 3, 3> m_icov_mat;

		/** \brief Eigen vectors of voxel covariance matrix */
		Eigen::Matrix<COMP_TYPE, 3, 3> m_eigen_vec; // Eigen vector of covariance matrix
		Eigen::Matrix<COMP_TYPE, 3, 1> m_eigen_val; // Eigen value of covariance values
		int m_last_eigen_decompose_size = 0;
		int m_create_frame_idx = 0;
		int m_last_update_frame_idx = 0;
		Feature_type m_feature_type = e_feature_sphere;
		double m_feature_determine_threshold_line = 1.0 / 3.0;
		double m_feature_determine_threshold_plane = 1.0 / 3.0;
		std::shared_ptr<Points_cloud_cell> m_previous_visited_cell = nullptr;
		Eigen::Matrix<COMP_TYPE, 3, 1> m_feature_vector;

	public:
		std::vector<PT_TYPE> m_points_vec;
		pcl::PointCloud<pcl_pt> m_pcl_pc_vec;
		DATA_TYPE m_cov_det_sqrt;
		bool m_mean_need_update = true;
		bool m_covmat_need_update = true;
		bool m_icovmat_need_update = true;
		size_t m_maximum_points_size = (size_t)1e3;
		int m_if_incremental_update_mean_and_cov = 0;
		std::shared_ptr<std::mutex> m_mutex_cell;
		double m_last_update_time;

		Points_cloud_cell();
		Points_cloud_cell(const PT_TYPE &cell_center, const DATA_TYPE &res = 1.0);
		~Points_cloud_cell();

		void set_data_need_update(int if_update_sum = 0);

		inline int get_points_count()
		{
			return m_points_vec.size();
		}

		inline Eigen::Matrix<DATA_TYPE, 3, 1> get_center()
		{
			return m_center.template cast<DATA_TYPE>();
		}

		Eigen::Matrix<DATA_TYPE, 3, 1> get_mean();

		void covmat_eig_decompose(int if_force_update = 0);

		Eigen::Matrix<COMP_TYPE, 3, 3> get_cov_mat_avoid_singularity();

		Eigen::Matrix<DATA_TYPE, 3, 3> get_covmat();

		Eigen::Matrix<DATA_TYPE, 3, 3> get_icovmat();

		pcl::PointCloud<pcl_pt> get_pointcloud();

		std::vector<PT_TYPE> get_pointcloud_eigen();

		void set_pointcloud(pcl::PointCloud<pcl_pt> &pc_in);

		void clear_data();

		void append_pt(const PT_TYPE &pt);
		void set_target_pc(const std::vector<PT_TYPE> &pt_vec);
		Feature_type determine_feature(int if_recompute = 0);
	};


	class Points_cloud_map
	{
	public:
		typedef Eigen::Matrix<DATA_TYPE, 3, 1> PT_TYPE;
		typedef Eigen::Matrix<DATA_TYPE, 3, 1> Eigen_Point;
		typedef Points_cloud_cell              Mapping_cell;
		typedef std::shared_ptr<Mapping_cell>  Mapping_cell_ptr;

		typedef std::unordered_map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Eigen_pt_hasher, PCL_TOOLS::Eigen_pt_compare> Map_pt_cell;
		typedef typename std::unordered_map<PT_TYPE, Mapping_cell_ptr, PCL_TOOLS::Eigen_pt_hasher, PCL_TOOLS::Eigen_pt_compare>::iterator Map_pt_cell_it;

		DATA_TYPE m_x_min, m_x_max;
		DATA_TYPE m_y_min, m_y_max;
		DATA_TYPE m_z_min, m_z_max;
		DATA_TYPE m_resolution;                    // resolution mean the distance of a cute to its bound.
		std::vector<Mapping_cell_ptr> m_cell_vec;
		int m_if_incremental_update_mean_and_cov = IF_ENABLE_INCREMENTAL_UPDATE_MEAN_COV;
		//std::unique_ptr< std::mutex >             m_mapping_mutex;
		std::shared_ptr<std::mutex> m_mapping_mutex;
		std::shared_ptr<std::mutex> m_octotree_mutex;
		std::shared_ptr<std::mutex> m_mutex_addcell;
		std::string m_json_file_name;
		std::vector<std::set<Mapping_cell_ptr>> m_frame_with_cell_index;
		float m_ratio_nonzero_line, m_ratio_nonzero_plane;
		int m_current_frame_idx;
		int m_minimum_revisit_threshold = std::numeric_limits<int>::max();

		Map_pt_cell m_map_pt_cell;               // using hash_map
		Map_pt_cell_it m_map_pt_cell_it;

		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> m_octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.0001);
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_pcl_cells_center = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		int m_initialized = false;

		Points_cloud_map()
		{
			m_mapping_mutex = std::make_shared<std::mutex>();
			m_mutex_addcell = std::make_shared<std::mutex>();
			m_octotree_mutex = std::make_shared<std::mutex>();
			m_x_min = std::numeric_limits<DATA_TYPE>::max();
			m_y_min = std::numeric_limits<DATA_TYPE>::max();
			m_z_min = std::numeric_limits<DATA_TYPE>::max();

			m_x_max = std::numeric_limits<DATA_TYPE>::min();
			m_y_max = std::numeric_limits<DATA_TYPE>::min();
			m_z_max = std::numeric_limits<DATA_TYPE>::min();
			m_pcl_cells_center->reserve(1e5);
			set_resolution(1.0);
			m_current_frame_idx = 0;
		};

		~Points_cloud_map()
		{
			m_mapping_mutex->try_lock();
			m_mapping_mutex->unlock();

			m_mutex_addcell->try_lock();
			m_mutex_addcell->unlock();

			m_octotree_mutex->try_lock();
			m_octotree_mutex->unlock();
		}

		inline void set_update_mean_and_cov_incrementally(int flag = 1)
		{
			m_if_incremental_update_mean_and_cov = flag;
		}
		inline int get_cells_size()
		{
			return m_map_pt_cell.size();
		}

		inline PT_TYPE find_cell_center(const PT_TYPE &pt)
		{
			PT_TYPE cell_center;
			DATA_TYPE box_size = m_resolution * 1.0;
			DATA_TYPE half_of_box_size = m_resolution * 0.5;

			// cell_center( 0 ) = ( std::round( ( pt( 0 ) - m_x_min - half_of_box_size ) / box_size ) ) * box_size + m_x_min + half_of_box_size;
			// cell_center( 1 ) = ( std::round( ( pt( 1 ) - m_y_min - half_of_box_size ) / box_size ) ) * box_size + m_y_min + half_of_box_size;
			// cell_center( 2 ) = ( std::round( ( pt( 2 ) - m_z_min - half_of_box_size ) / box_size ) ) * box_size + m_z_min + half_of_box_size;

			cell_center(0) = (std::round((pt(0) - half_of_box_size) / box_size)) * box_size + half_of_box_size;
			cell_center(1) = (std::round((pt(1) - half_of_box_size) / box_size)) * box_size + half_of_box_size;
			cell_center(2) = (std::round((pt(2) - half_of_box_size) / box_size)) * box_size + half_of_box_size;

			return cell_center;
		}

		inline void clear_data()
		{
			for (Map_pt_cell_it it = m_map_pt_cell.begin(); it != m_map_pt_cell.end(); it++)
			{
				it->second->clear_data();
			}
			m_map_pt_cell.clear();
			m_cell_vec.clear();
			std::vector<Mapping_cell_ptr>().swap(m_cell_vec);
			m_pcl_cells_center->clear();
			m_octree.deleteTree();
		}

		inline void set_point_cloud(const std::vector<PT_TYPE> &input_pt_vec, std::set<std::shared_ptr<Mapping_cell>> *cell_vec = nullptr)
		{
			clear_data();
			for (size_t i = 0; i < input_pt_vec.size(); i++)
			{
				m_x_min = std::min(input_pt_vec[i](0), m_x_min);
				m_y_min = std::min(input_pt_vec[i](1), m_y_min);
				m_z_min = std::min(input_pt_vec[i](2), m_z_min);

				m_x_max = std::max(input_pt_vec[i](0), m_x_max);
				m_y_max = std::max(input_pt_vec[i](1), m_y_max);
				m_z_max = std::max(input_pt_vec[i](2), m_z_max);
			}
			if (cell_vec != nullptr)
			{
				cell_vec->clear();
			}
			for (size_t i = 0; i < input_pt_vec.size(); i++)
			{
				std::shared_ptr<Mapping_cell> cell = find_cell(input_pt_vec[i], 1, 1);
				cell->append_pt(input_pt_vec[i]);
				if (cell_vec != nullptr)
				{
					cell_vec->insert(cell);
				}
			}

			m_octree.setInputCloud(m_pcl_cells_center);
			m_octree.addPointsFromInputCloud();
			std::cout << "*** set_point_cloud octree initialization finish ***" << std::endl;
			m_initialized = true;
			m_current_frame_idx++;
		}

		inline void append_cloud(const std::vector<PT_TYPE> &input_pt_vec, std::set<Mapping_cell_ptr> *cell_vec = nullptr)
		{
			// ENABLE_SCREEN_PRINTF;
			std::map<Mapping_cell_ptr, int> appeared_cell_count;
			m_mapping_mutex->lock();
			int current_size = get_cells_size();
			if (current_size == 0)
			{
				set_point_cloud(input_pt_vec, cell_vec);
				m_mapping_mutex->unlock();
			}
			else
			{
				m_mapping_mutex->unlock();
				if (cell_vec != nullptr)
				{
					cell_vec->clear();
				}
				for (size_t i = 0; i < input_pt_vec.size(); i++)
				{
					Mapping_cell_ptr cell = find_cell(input_pt_vec[i], 1, 1);
					cell->append_pt(input_pt_vec[i]);
					if (cell_vec != nullptr)
					{
						auto it = appeared_cell_count.find(cell);
						if (it != appeared_cell_count.end())
						{
							it->second++;
						}
						else
						{
							appeared_cell_count.insert(std::make_pair(cell, 1));
						}
					}
				}

				if (cell_vec != nullptr)
				{
					for (auto it = appeared_cell_count.begin(); it != appeared_cell_count.end(); it++)
					{
						if (it->second >= 3)
						{
							cell_vec->insert(it->first);
						}
					}
				}
			}
			m_current_frame_idx++;
			std::cout << "Input points size: "   << input_pt_vec.size() << ", "
					   << "add cell number: "    << get_cells_size() - current_size << ", "
					   << "curren cell number: " << m_map_pt_cell.size() << std::endl;
		}

		template <typename T>
		inline void set_resolution(T resolution)
		{
			m_resolution = DATA_TYPE(resolution * 0.5);
			std::cout << "Resolution is set as: " << m_resolution << std::endl;
			m_octree.setResolution(m_resolution);
		};

		inline DATA_TYPE get_resolution()
		{
			return m_resolution * 2;
		}

		inline Mapping_cell_ptr add_cell(const PT_TYPE &cell_center)
		{
			std::unique_lock<std::mutex> lock(*m_mutex_addcell);
			Map_pt_cell_it it = m_map_pt_cell.find(cell_center);
			if (it != m_map_pt_cell.end())
			{
				return it->second;
			}

			Mapping_cell_ptr cell = std::make_shared<Mapping_cell>(cell_center, (DATA_TYPE)m_resolution);
			cell->m_create_frame_idx = m_current_frame_idx;
			cell->m_last_update_frame_idx = m_current_frame_idx;
			cell->m_if_incremental_update_mean_and_cov = m_if_incremental_update_mean_and_cov;
			m_map_pt_cell.insert(std::make_pair(cell_center, cell));

			if (m_initialized == false)
			{
				m_pcl_cells_center->push_back(pcl::PointXYZ(cell->m_center(0), cell->m_center(1), cell->m_center(2)));
			}
			else
			{
				std::unique_lock<std::mutex> lock(*m_octotree_mutex);
				m_octree.addPointToCloud(pcl::PointXYZ(cell->m_center(0), cell->m_center(1), cell->m_center(2)), m_pcl_cells_center);
			}

			m_cell_vec.push_back(cell);
			return cell;
		}

		inline Mapping_cell_ptr find_cell(const PT_TYPE &pt, int if_add = 1, int if_treat_revisit = 0)
		{
			PT_TYPE cell_center = find_cell_center(pt);
			Map_pt_cell_it it = m_map_pt_cell.find(cell_center);
			if (it == m_map_pt_cell.end())
			{
				if (if_add)
				{
					Mapping_cell_ptr cell_ptr = add_cell(cell_center);
					return cell_ptr;
				}
				else
				{
					return nullptr;
				}
			}
			else
			{
				if (if_treat_revisit)
				{
					if (m_current_frame_idx - it->second->m_last_update_frame_idx < m_minimum_revisit_threshold)
					{
						it->second->m_last_update_frame_idx = m_current_frame_idx;
						return it->second;
					}
					else
					{
						// Avoid confilcts of revisited
						// ENABLE_SCREEN_PRINTF;
						// screen_out << "!!!!! Cell revisit, curr_idx = " << m_current_frame_idx << " ,last_idx = " << it->second->m_last_update_frame_idx << std::endl;

						Mapping_cell_ptr new_cell = std::make_shared<Mapping_cell>(it->second->get_center(), (DATA_TYPE)m_resolution);
						m_cell_vec.push_back(new_cell);
						new_cell->m_previous_visited_cell = it->second;
						it->second = new_cell;
						it->second->m_create_frame_idx = m_current_frame_idx;
						it->second->m_last_update_frame_idx = m_current_frame_idx;
						//screen_out << ", find cell addr = " << ( void * ) find_cell( pt ) << std::endl;
					}
				}
				return it->second;
			}
		}

		template <typename T>
		inline std::vector<Mapping_cell_ptr> find_cells_in_radius(T pt, float searchRadius = 0)
		{
			std::unique_lock<std::mutex> lock(*m_octotree_mutex);

			std::vector<Mapping_cell_ptr> cells_vec;
			pcl::PointXYZ searchPoint = PCL_TOOLS::eigen_to_pcl_pt<pcl::PointXYZ>(pt);
			std::vector<int> cloudNWRSearch;
			std::vector<float> cloudNWRRadius;

			if (searchRadius == 0)
			{
				m_octree.radiusSearch(searchPoint, m_resolution, cloudNWRSearch, cloudNWRRadius);
			}
			else
			{
				m_octree.radiusSearch(searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);
			}

			PT_TYPE eigen_pt;
			for (size_t i = 0; i < cloudNWRSearch.size(); i++)
			{

				eigen_pt = PCL_TOOLS::pcl_pt_to_eigen<DATA_TYPE>(m_octree.getInputCloud()->points[cloudNWRSearch[i]]);
				cells_vec.push_back(find_cell(eigen_pt));
			}

			return cells_vec;
		}


		inline pcl::PointCloud<pcl_pt> get_all_pointcloud()
		{
			pcl::PointCloud<pcl_pt> res_pt;
			for (size_t i = 0; i < m_cell_vec.size(); i++)
			{

				res_pt += m_cell_vec[i]->get_pointcloud();
			}
			return res_pt;
		}
	};

}

#endif //BALM_CLASS_H