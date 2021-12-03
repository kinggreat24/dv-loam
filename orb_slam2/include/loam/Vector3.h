/*
 * @Author: your name
 * @Date: 2020-04-09 10:44:38
 * @LastEditTime: 2020-05-21 16:59:55
 * @LastEditors: Kinggreat24
 * @Description: In User Settings Edit
 * @FilePath: /ORB_SLAM2-master/include/Vector3.h
 */
#ifndef ORB_SLAM2_VECTOR3_H
#define ORB_SLAM2_VECTOR3_H


#include <pcl/point_types.h>


namespace loam {

/** \brief Vector4f decorator for convenient handling.
 *
 */
class Vector3 : public Eigen::Vector4f {
public:
  Vector3(float x, float y, float z)
      : Eigen::Vector4f(x, y, z, 0) {}

  Vector3(void)
      : Eigen::Vector4f(0, 0, 0, 0) {}

  template<typename OtherDerived>
  Vector3(const Eigen::MatrixBase <OtherDerived> &other)
      : Eigen::Vector4f(other) {}

  Vector3(const pcl::PointXYZI &p)
      : Eigen::Vector4f(p.x, p.y, p.z, 0) {}

  template<typename OtherDerived>
  Vector3 &operator=(const Eigen::MatrixBase <OtherDerived> &rhs) {
    this->Eigen::Vector4f::operator=(rhs);
    return *this;
  }

  Vector3 &operator=(const pcl::PointXYZ &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  Vector3 &operator=(const pcl::PointXYZI &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  float x() const { return (*this)(0); }

  float y() const { return (*this)(1); }

  float z() const { return (*this)(2); }

  float &x() { return (*this)(0); }

  float &y() { return (*this)(1); }

  float &z() { return (*this)(2); }

  // easy conversion
  operator pcl::PointXYZI() {
    pcl::PointXYZI dst;
    dst.x = x();
    dst.y = y();
    dst.z = z();
    dst.intensity = 0;
    return dst;
  }
};

} // end namespace ORB_SLAM2

#endif //ORB_SLAM2_VECTOR3_H
