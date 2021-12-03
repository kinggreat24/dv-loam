#ifndef __PCL_TOOLS_HPP__
#define __PCL_TOOLS_HPP__
#include <iostream>
#include <stdio.h>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

#define min_f(a, b, c) (fminf(a, fminf(b, c)))
#define max_f(a, b, c) (fmaxf(a, fmaxf(b, c)))

namespace PCL_TOOLS
{
    using namespace std;
    // using namespace Common_tools;
    struct Pt_compare
    {
        //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
        template <typename _T>
        inline bool operator()(const _T &a, const _T &b)
        {
            return ((a.x < b.x) || (a.x == b.x && a.y < b.y) || ((a.x == b.x) && (a.y == b.y) && (a.z < b.z)));
        }

        template <typename _T>
        bool operator()(const _T &a, const _T &b) const
        {
            return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
        }
    };

    struct Pt_hasher
    {
        template <typename _T>
        std::size_t operator()(const _T &k) const
        {
            return ((std::hash<float>()(k.x) ^ (std::hash<float>()(k.y) << 1)) >> 1) ^ (std::hash<float>()(k.z) << 1);
        }
    };

    struct Eigen_pt_compare
    {
        //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
        template <typename _T>
        inline bool operator()(const _T &a, const _T &b)
        {
            return ((a(0) < b(0)) || (a(0) == b(0) && a(1) < b(1)) || ((a(0) == b(0)) && (a(1) == b(1)) && (a(2) < b(2))));
        }

        template <typename _T>
        bool operator()(const _T &a, const _T &b) const
        {
            return (a(0) == b(0)) && (a(1) == b(1)) && (a(2) == b(2));
        }
    };

    struct Eigen_pt_hasher
    {
        template <typename _T>
        std::size_t operator()(const _T &k) const
        {
            return ((std::hash<float>()(k(0)) ^ (std::hash<float>()(k(1)) << 1)) >> 1) ^ (std::hash<float>()(k(2)) << 1);
        }
    };

    template <typename TT, typename PointType>
    Eigen::Matrix<TT, 3, 1> pcl_pt_to_eigen(const PointType &pt)
    {
        return Eigen::Matrix<TT, 3, 1>((TT)pt.x, (TT)pt.y, (TT)pt.z);
    }

    template <typename T>
    Eigen::Matrix<float, 3, 1> pcl_pt_to_eigenf(const T &pt)
    {
        return pcl_pt_to_eigen<float>(pt);
    }

    template <typename T>
    Eigen::Matrix<double, 3, 1> pcl_pt_to_eigend(const T &pt)
    {
        return pcl_pt_to_eigen<double>(pt);
    }

    template <typename TT, typename T>
    TT eigen_to_pcl_pt(const T &pt)
    {
        TT res_pt;
        res_pt.x = pt(0);
        res_pt.y = pt(1);
        res_pt.z = pt(2);
        return res_pt;
    }

    template <typename PointType, typename T>
    pcl::PointCloud<PointType> eigen_pt_to_pcl_pointcloud(const vector<T> &eigen_pt_vec)
    {
        pcl::PointCloud<PointType> pcl_pc_vec;
        pcl_pc_vec.resize(eigen_pt_vec.size());
        for (size_t i = 0; i < eigen_pt_vec.size(); i++)
        {
            pcl_pc_vec[i] = eigen_to_pcl_pt<PointType>(eigen_pt_vec[i]);
        }
        return pcl_pc_vec;
    }

    template <typename T, typename PointType>
    pcl::PointCloud<PointType> pointcloud_transfrom(pcl::PointCloud<PointType> pcl_pt_in, Eigen::Matrix<T, 3, 3> tranfrom_R, Eigen::Matrix<T, 3, 1> tranfrom_T)
    {
        pcl::PointCloud<PointType> pcl_pc_res;
        pcl_pc_res.resize(pcl_pt_in.size());
        //cout << "Pointcloud_transfrom_T: \r\n " << tranfrom_T.transpose() << endl;
        //cout << "Pointcloud_transfrom_R: \r\n " << tranfrom_R << endl;
        for (size_t i = 0; i < pcl_pt_in.size(); i++)
        {
            auto eigen_pt = PCL_TOOLS::pcl_pt_to_eigen<T, PointType>(pcl_pt_in.points[i]);
            pcl_pc_res.points.push_back(PCL_TOOLS::eigen_to_pcl_pt<PointType>(tranfrom_R * eigen_pt + tranfrom_T));
        }
        return pcl_pc_res;
    }

   
   
    class PCL_point_cloud_to_pcd
    {
    public:
        string m_save_dir_name;
        string m_save_file_name;
        int m_save_files_index;

        ~PCL_point_cloud_to_pcd(){};
        PCL_point_cloud_to_pcd()
        {
            m_save_dir_name = "";
            m_save_files_index = 0;
        };

        template <typename T>
        void save_to_pcd_files(string file_name, T &cloud, int index = 0)
        {
            char tempchar[5000];

            if (index == 0)
            {

                sprintf(tempchar, "%s/%s_%d.pcd", m_save_dir_name.c_str(), file_name.c_str(), m_save_files_index);
                m_save_files_index++;
            }
            else
            {

                sprintf(tempchar, "%s/%s_%d.pcd", m_save_dir_name.c_str(), file_name.c_str(), index);
                cout << "----- Save cloud to " << tempchar << " -----" << endl;
            }

            pcl::io::savePCDFile(tempchar, cloud);

        };
    };

    template <typename T>
    int load_from_pcd_file(const char *file_name, pcl::PointCloud<T> &target_cloud)
    {
        if (pcl::io::loadPCDFile<T>(file_name, target_cloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", file_name);
            return (0);
        }
        else
        {
            cout << "Get points number:" << target_cloud.size() << endl;
            return 1;
        }
    };

    void rgb2hsv(const unsigned char &src_r, const unsigned char &src_g, const unsigned char &src_b, unsigned char &dst_h, unsigned char &dst_s, unsigned char &dst_v);
    void hsv2rgb(const unsigned char &src_h, const unsigned char &src_s, const unsigned char &src_v, unsigned char &dst_r, unsigned char &dst_g, unsigned char &dst_b);
    pcl::PointCloud<pcl::PointXYZRGBA> PCL_XYZI_to_RGBA(pcl::PointCloud<pcl::PointXYZI> &pt_in, float alpha = 0.1);

    template <typename T, typename PointType>
    vector<Eigen::Matrix<T, 3, 1>> pcl_pts_to_eigen_pts(const typename pcl::PointCloud<PointType>::Ptr input_cloud)
    {
        vector<Eigen::Matrix<T, 3, 1>> out_res;
        size_t pc_size = input_cloud->size();
        out_res.resize(pc_size);
        for (size_t i = 0; i < pc_size; i++)
        {
            //out_res[ i ] << input_cloud->points[ i ]x, input_cloud->points[ i ].y, input_cloud->points[ i ].z;
            out_res[i] << input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z;
        }
        return out_res;
    }

} // namespace PCL_TOOLS
#endif
