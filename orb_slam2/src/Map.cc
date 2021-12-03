/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include <mutex>

namespace ORB_SLAM2
{

    Map::Map() 
        : mnMaxKFid(0), mnBigChangeIdx(0), mfCornerMapRes(0.3), mfSurfMapRes(0.5), mfGroundRes(0.8), mbShutdownFlag(false)
    {
        mnLastBigChangeIdx = 0;
        mnLastKeyFrameIdx = 0;

        mDownSizeFilterCorner.setLeafSize(mfCornerMapRes, mfCornerMapRes, mfCornerMapRes);
        mDownSizeFilterSurf.setLeafSize(mfSurfMapRes, mfSurfMapRes, mfSurfMapRes);
        mDownSizeFilterGround.setLeafSize(mfGroundRes, mfGroundRes, mfGroundRes);

        //格网地图初始化
        m_pt_corner_cell_resolution = 0.3;
        m_pt_surf_cell_resolution   = 0.6;
        m_pt_cell_map_corner.set_resolution(m_pt_corner_cell_resolution);
        m_pt_cell_map_corner.m_minimum_revisit_threshold = 2000;
        m_pt_cell_map_surf.set_resolution(m_pt_surf_cell_resolution);
        m_pt_cell_map_surf.m_minimum_revisit_threshold = 2000;

        // mpLocalLidarMapThread = std::make_shared<std::thread>(std::bind(&Map::UpdateLocalMapThread, this));
    }

    Map::~Map()
    {
        mbShutdownFlag = true;

        if(mpLocalLidarMapThread)
            mpLocalLidarMapThread->join();
    }

    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);

        mKeyFrameUpdated.notify_one();

        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;

        mKeyFrameUpdated.notify_one();
    }

    int Map::GetLastBigChangeIdx()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::GetConnectedKeyFrames(KeyFrame* pKF, std::vector<KeyFrame*>& connected_keyframe, int n) 
    {
        // Sophus::SE3f Twc_ref_KF = Converter::toSE3f(pKF->GetPoseInverse());
        
        // mvpKeyFrames = GetAllKeyFrames();
        // sort(mvpKeyFrames.begin(), mvpKeyFrames.end(), KeyFrame::lId);
        // if(n > mvpKeyFrames.size())
        //     n = mvpKeyFrames.size();
            
        // for(int i = n; i > 1; --i) 
        // {
        //     KeyFrame* pCurKf = mvpKeyFrames[i-1];
            
        //     //计算关键帧的距离，以及角度差
        //     Sophus::SE3f Twc_cur_KF = Converter::toSE3f(pKF->GetPoseInverse());
        //     Eigen::Vector3f dist = Twc_ref_KF.translation() - Twc_cur_KF.translation();
        //     if(dist.norm() > 30)
        //         continue;

        //     //计算关键帧的视角差
        
        //     //加入到列表中
        //     connected_keyframe.push_back(pCurKf);
        // }
    }

    void Map::clear()
    {
        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
            delete *sit;

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    // 将当前帧特征点云添加到格网地图中
    void Map::InsertCurrentFeaturePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCornerPoints,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr pSurfPoints,
                                       const Eigen::Matrix4d &Twc)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pSurfPoints_world(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pCornerPoints_world(new pcl::PointCloud<pcl::PointXYZI>());
        
        //添加边缘点格网地图
        std::cout<<"Add corner cell map"<<std::endl;
        pcl::transformPointCloud(*pCornerPoints, *pCornerPoints_world, Twc);
        m_pt_cell_map_corner.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, pcl::PointXYZI>(pCornerPoints_world));

        //添加平坦点格网地图
        std::cout<<"Add surf cell map"<<std::endl;
        pcl::transformPointCloud(*pSurfPoints, *pSurfPoints_world, Twc);
        m_pt_cell_map_surf.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, pcl::PointXYZI>(pSurfPoints_world));
    }


    void Map::GetLocalCellMap(const Eigen::Matrix4d &Twc, const float radius,
        pcl::PointCloud<pcl::PointXYZI>::Ptr pCornerMap,
        pcl::PointCloud<pcl::PointXYZI>::Ptr pSurfMap)
    {
        unique_lock<mutex> lck_keyframeUpdated(mMutexMap);

        Eigen::Vector3d pos = Twc.block<3, 1>(0, 3);

        std::vector<Points_cloud_map::Mapping_cell_ptr> cell_vec = m_pt_cell_map_surf.find_cells_in_radius(pos, radius);
        for (size_t i = 0; i < cell_vec.size(); i++)
        {
            *pSurfMap += cell_vec[i]->get_pointcloud();
        }

        std::vector<Points_cloud_map::Mapping_cell_ptr> inliner_corner_cell_vec = 
                m_pt_cell_map_corner.find_cells_in_radius(pos,radius);
        for (size_t i = 0; i < inliner_corner_cell_vec.size(); i++)
        {
            *pCornerMap += inliner_corner_cell_vec[i]->get_pointcloud();
        }
    }


    void Map::UpdateLocalMapThread()
    {
        while (!mbShutdownFlag)
        {
            {
                unique_lock<mutex> lck_keyframeUpdated(mMutexMap);
                mKeyFrameUpdated.wait(lck_keyframeUpdated);
            }
            
            size_t N = 0;
            {
                mvpKeyFrames = GetAllKeyFrames();
                sort(mvpKeyFrames.begin(), mvpKeyFrames.end(), KeyFrame::lId);
                N = mvpKeyFrames.size();
            }

            // pose graph 优化后，更新所有关键帧的姿态，重新生成局部点云地图
            if (mnLastBigChangeIdx != mnBigChangeIdx)         
            {
                mnLastKeyFrameIdx = 0;                       // 更新所有的点云
                mnLastBigChangeIdx = mnBigChangeIdx;
            
                m_pt_cell_map_corner.clear_data();
            }

            if (mnLastKeyFrameIdx == N) // 没有新的关键帧插入
                continue;

            // 将新的关键帧添加到格网地图中，同时更新格网的特征值
            for (size_t j = mnLastKeyFrameIdx; j < mvpKeyFrames.size(); ++j)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr transform_pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
                Eigen::Matrix4d Twc = Converter::toMatrix4d(mvpKeyFrames[j]->GetPoseInverse()).matrix();
                InsertCurrentFeaturePointsToMap(mvpKeyFrames[j]->mpCornerPointsLessSharp, 
                    mvpKeyFrames[j]->mpSurfPointsLessFlat,Twc);
            }
            mnLastKeyFrameIdx = N;

            // 保留一定区域内的点云
            // Eigen::Matrix4d Twc      = Converter::toMatrix4d(mvpKeyFrames[N-1]->GetPoseInverse()).matrix();
            // Eigen::Vector3d position = Twc.block<3,1>(0,3);
            // std::vector<Points_cloud_map::Mapping_cell_ptr> inliner_cell_vec = 
            //     m_pt_cell_map_surf.find_cells_in_radius(position,100);
            
            // pcl::PointCloud<pcl::PointXYZI> plane_cell_pointcloud;
            // for ( size_t cell_idx = 0; cell_idx < inliner_cell_vec.size(); cell_idx++ )
            // {
            //     Feature_type cell_feature_type = inliner_cell_vec[ cell_idx ]->determine_feature();
            //     if(cell_feature_type == e_feature_plane)
            //     {
            //         pcl::PointCloud<pcl::PointXYZI> cell_pointcloud = inliner_cell_vec[ cell_idx ]->get_pointcloud();
            //         for(int i=0;i<cell_pointcloud.size();i++)
            //             cell_pointcloud.points[i].intensity = cell_idx;

            //         plane_cell_pointcloud += cell_pointcloud;
            //     }
            // }
            // m_pt_cell_map_surf.clear_data();
            // m_pt_cell_map_surf.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, pcl::PointXYZI>(plane_cell_pointcloud.makeShared()));


            // std::vector<Points_cloud_map::Mapping_cell_ptr> inliner_corner_cell_vec = 
            //     m_pt_cell_map_corner.find_cells_in_radius(position,100);
            // pcl::PointCloud<pcl::PointXYZI> corner_cell_pointcloud;
            
            // for ( size_t cell_idx = 0; cell_idx < inliner_corner_cell_vec.size(); cell_idx++ )
            // {
            //     Feature_type cell_feature_type = inliner_corner_cell_vec[ cell_idx ]->determine_feature();
            //     if(/*cell_feature_type == e_feature_line*/true)
            //     {
            //         pcl::PointCloud<pcl::PointXYZI> cell_pointcloud = inliner_corner_cell_vec[ cell_idx ]->get_pointcloud();
            //         for(int i=0;i<cell_pointcloud.size();i++)
            //             cell_pointcloud.points[i].intensity = cell_idx;

            //         corner_cell_pointcloud += cell_pointcloud;
            //     }
            // }

            // m_pt_cell_map_corner.clear_data();
            // m_pt_cell_map_corner.append_cloud(PCL_TOOLS::pcl_pts_to_eigen_pts<float, pcl::PointXYZI>(corner_cell_pointcloud.makeShared()));

            // char file_name[128]={0};
            // sprintf(file_name,"/home/kinggreat24/pc/%d_plane.pcd",N);
            // pcl::io::savePCDFileASCII(file_name, plane_cell_pointcloud);//将点云保存到PCD文件中
            // sprintf(file_name,"/home/kinggreat24/pc/%d_corner.pcd",N);
            // pcl::io::savePCDFileASCII(file_name, corner_cell_pointcloud);//将点云保存到PCD文件中


            // 遍历所有的cell的信息
            // for(size_t cell_idx = 0; cell_idx < m_pt_cell_map_corner.m_map_pt_cell.size(); cell_idx++)
            // {
            //     PT_TYPE cell_center = PCL_TOOLS::pcl_pt_to_eigen<DATA_TYPE>(m_pt_cell_map_corner.m_octree.getInputCloud()->points[cell_idx]);
            //     Points_cloud_map::Map_pt_cell_it it = m_pt_cell_map_corner.m_map_pt_cell.find(cell_center);
            //     if(it != m_pt_cell_map_corner.m_map_pt_cell.end())
            //     {
            //         Points_cloud_map::Mapping_cell_ptr cell = it->second;
            //         cell->get_mean();
            //     }
            // }
           

            // sleep 50ms
            usleep(5000);
        }
    }

    // map serialization addition
    template <class Archive>
    void Map::serialize(Archive &ar, const unsigned int version)
    {
        // don't save mutex
        unique_lock<mutex> lock_MapUpdate(mMutexMapUpdate);
        unique_lock<mutex> lock_Map(mMutexMap);
        ar &mspMapPoints;
        ar &mvpKeyFrameOrigins;
        ar &mspKeyFrames;
        ar &mvpReferenceMapPoints;
        ar &mnMaxKFid &mnBigChangeIdx;
    }
    template void Map::serialize(boost::archive::binary_iarchive &, const unsigned int);
    template void Map::serialize(boost::archive::binary_oarchive &, const unsigned int);

} //namespace ORB_SLAM
