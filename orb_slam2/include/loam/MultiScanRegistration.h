// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#ifndef ORB_SLAM2_MULTISCANREGISTRATION_H
#define ORB_SLAM2_MULTISCANREGISTRATION_H


#include "loam/BasicScanRegistration.h"

// #include <sensor_msgs/PointCloud2.h>


namespace loam {

/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring.
 *
 */
class MultiScanMapper {
public:
  /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  MultiScanMapper(const float& lowerBound = -15,
                  const float& upperBound = 15,
                  const uint16_t& nScanRings = 16);

  const float& getLowerBound() { return _lowerBound; }
  const float& getUpperBound() { return _upperBound; }
  const uint16_t& getNumberOfScanRings() { return _nScanRings; }

  /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  void set(const float& lowerBound,
           const float& upperBound,
           const uint16_t& nScanRings);

  /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
  int getRingForAngle(const float& angle);

  /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
  static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

  /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };

  /** Multi scan mapper for CFANS CFANS-16 according to data sheet. */
  static inline MultiScanMapper CFANS_16() { return MultiScanMapper(-15, 15, 16);  };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper CFANS_32() { return MultiScanMapper(-10.2f, 8.4, 32); };


private:
  float _lowerBound;      ///< the vertical angle of the first scan ring
  float _upperBound;      ///< the vertical angle of the last scan ring
  uint16_t _nScanRings;   ///< number of scan rings
  float _factor;          ///< linear interpolation factor
};



/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class MultiScanRegistration : virtual public BasicScanRegistration {
public:
  MultiScanRegistration(const MultiScanMapper& scanMapper = MultiScanMapper());


  /** \brief Handler method for IMU messages.
     *
     * @param imuIn the new IMU message
     */
  void handleIMUMessage(const Imu& imuIn);


  /** \brief Handler method for input cloud messages.
   *
   * @param laserCloudIn the new input cloud message to process
   * @param scanTime                  the time of new input cloud message
   * @param cornerPointsSharpOut      the cornet points of the cloud
   * @param cornerPointsLessSharpOut  the less cornet points of the cloud
   * @param surfacePointsFlatOut      the surf points of the cloud
   * @param surfacePointsLessFlatOut  the less surf points of the cloud
   */
  void handleCloudMessage(const pcl::PointCloud<pcl::PointXYZI>& laserCloudIn,
                          const Time& scanTime,
                          pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharpOut,
                          pcl::PointCloud<pcl::PointXYZI>& cornerPointsLessSharpOut,
                          pcl::PointCloud<pcl::PointXYZI>& surfacePointsFlatOut,
                          pcl::PointCloud<pcl::PointXYZI>& surfacePointsLessFlatOut);

  /** \brief Setup component in active mode.
   * 
   */
  bool setupRegistrationParams(const std::string& lidarName, const RegistrationParams& config, const float vAngleMin=0, const float vAngleMax=15, const int rings=16);



  void publishResult(pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharp,
                     pcl::PointCloud<pcl::PointXYZI>& cornerPointsLessSharp,
                     pcl::PointCloud<pcl::PointXYZI>& surfacePointsFlat,
                     pcl::PointCloud<pcl::PointXYZI>& surfacePointsLessFlat);
private:
  /** \brief Process a new input cloud.
   *
   * @param laserCloudIn the new input cloud to process
   * @param scanTime the scan (message) timestamp
   */
  void process(const pcl::PointCloud<pcl::PointXYZI>& laserCloudIn, const Time& scanTime);

private:
  int _systemDelay = 20;                        ///< system startup delay counter
  MultiScanMapper _scanMapper;                  ///< mapper for mapping vertical point angles to scan ring IDs
  RegistrationParams _registrationParams;
  std::vector<pcl::PointCloud<pcl::PointXYZI> > _laserCloudScans;
};

} // end namespace loam


#endif //ORB_SLAM2_MULTISCANREGISTRATION_H
