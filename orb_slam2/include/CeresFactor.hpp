// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#ifndef CERES_FACTOR_H
#define CERES_FACTOR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/cubic_interpolation.h>

#include "assert.h"

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vikit/pinhole_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/patch_score.h>

inline Eigen::Matrix<double, 3, 3> skew(Eigen::Matrix<double, 3, 1> &mat_in)
{
	Eigen::Matrix<double, 3, 3> skew_mat;
	skew_mat.setZero();
	skew_mat(0, 1) = -mat_in(2);
	skew_mat(0, 2) = mat_in(1);
	skew_mat(1, 2) = -mat_in(0);
	skew_mat(1, 0) = mat_in(2);
	skew_mat(2, 0) = -mat_in(1);
	skew_mat(2, 1) = mat_in(0);
	return skew_mat;
}

inline void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t)
{
	Eigen::Vector3d omega(se3.data());
	Eigen::Vector3d upsilon(se3.data() + 3);
	Eigen::Matrix3d Omega = skew(omega);

	double theta = omega.norm();
	double half_theta = 0.5 * theta;

	double imag_factor;
	double real_factor = cos(half_theta);
	if (theta < 1e-10)
	{
		double theta_sq = theta * theta;
		double theta_po4 = theta_sq * theta_sq;
		imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
	}
	else
	{
		double sin_half_theta = sin(half_theta);
		imag_factor = sin_half_theta / theta;
	}

	q = Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());

	Eigen::Matrix3d J;
	if (theta < 1e-10)
	{
		J = q.matrix();
	}
	else
	{
		Eigen::Matrix3d Omega2 = Omega * Omega;
		J = (Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * Omega + (theta - sin(theta)) / (pow(theta, 3)) * Omega2);
	}

	t = J * upsilon;
}


/********************************************************************************/
/*                  　 激光点误差（边缘点到线的误差）                                */
/********************************************************************************/
struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d ref_point_, Eigen::Vector3d cur_point_a_,
					Eigen::Vector3d cur_point_b_, double s_)
		: ref_point(ref_point_), cur_point_a(cur_point_a_), cur_point_b(cur_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> rp{T(ref_point.x()), T(ref_point.y()), T(ref_point.z())};
		Eigen::Matrix<T, 3, 1> cpa{T(cur_point_a.x()), T(cur_point_a.y()), T(cur_point_a.z())};
		Eigen::Matrix<T, 3, 1> cpb{T(cur_point_b.x()), T(cur_point_b.y()), T(cur_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_cur_ref{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_cur_ref = q_identity.slerp(T(s), q_cur_ref);
		Eigen::Matrix<T, 3, 1> t_cur_ref{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> cp;
		cp = q_cur_ref * rp + t_cur_ref;

		Eigen::Matrix<T, 3, 1> nu = (cp - cpa).cross(cp - cpb);
		Eigen::Matrix<T, 3, 1> de = cpa - cpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d ref_point_, const Eigen::Vector3d cur_point_a_,
									   const Eigen::Vector3d cur_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(ref_point_, cur_point_a_, cur_point_b_, s_)));
	}

	Eigen::Vector3d ref_point, cur_point_a, cur_point_b;
	double s;
};

//平坦点到平面的误差
struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		//Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
		//Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

//平面法相误差
struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};

struct LidarDistanceFactor
{

	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_)
		: curr_point(curr_point_), closed_point(closed_point_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d closed_point;
};

//2D重投影误差
struct SnavelyReprojectionFactorPoseOnly
{
	SnavelyReprojectionFactorPoseOnly(Eigen::Vector3d pw_, double observed_x_, double observed_y_, vk::PinholeCamera *pinhole_cam_)
		: pw(pw_), observed_x(observed_x_), observed_y(observed_y_), pinhole_cam(pinhole_cam_)
	{
		fx = pinhole_cam->fx();
		fy = pinhole_cam->fy();
		cx = pinhole_cam->cx();
		cy = pinhole_cam->cy();
		width = pinhole_cam->width();
		height = pinhole_cam->height();
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 3> R_curr_w = q_w_curr.toRotationMatrix().transpose();
		Eigen::Matrix<T, 3, 1> t_curr_w = -R_curr_w * t_w_curr;

		Eigen::Matrix<T, 3, 1> pt_w{T(pw[0]), T(pw[1]), T(pw[2])};
		Eigen::Matrix<T, 3, 1> pc = R_curr_w * pt_w + t_curr_w;

		if (pc.z() <= T(0))
			return false;

		T u = pc[0] / pc[2] * T(pinhole_cam->fx()) + T(pinhole_cam->cx());
		T v = pc[1] / pc[2] * T(pinhole_cam->fy()) + T(pinhole_cam->cy());

		residual[0] = T(observed_x) - u;
		residual[1] = T(observed_y) - v;

		return true;
	}

	static ceres::CostFunction *Create(Eigen::Vector3d pw_, double observed_x_, double observed_y_, vk::PinholeCamera *pinhole_cam_)
	{
		return (new ceres::AutoDiffCostFunction<
				SnavelyReprojectionFactorPoseOnly, 2, 4, 3>(
			new SnavelyReprojectionFactorPoseOnly(pw_, observed_x_, observed_y_, pinhole_cam_)));
	}

	vk::PinholeCamera *pinhole_cam;
	Eigen::Vector3d pw; //世界系坐标
	double observed_x;
	double observed_y;

	int width;
	int height;
	double fx, fy, cx, cy;
};

//直接法
struct PhotometricFactorPoseOnly
{
	PhotometricFactorPoseOnly(Eigen::Vector3d pl_, float pixel_value_, vk::PinholeCamera *pinhole_cam_, ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 1>> &interpolator_)
		: pl(pl_), pixel_value(pixel_value_), pinhole_cam(pinhole_cam_), interpolator(interpolator_)
	{
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_curr_last{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_curr_last{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> pl_{T(pl[0]), T(pl[1]), T(pl[2])};
		Eigen::Matrix<T, 3, 1> pc = q_curr_last * pl_ + t_curr_last;

		if (pc.z() <= T(0))
			return false;

		T u = pc[0] / pc[2] * T(pinhole_cam->fx()) + T(pinhole_cam->cx());
		T v = pc[1] / pc[2] * T(pinhole_cam->fy()) + T(pinhole_cam->cy());

		if (isInImage<T>(u, v))
			return false;

		T pixel_projected = T(0);
		interpolator.Evaluate(v, u, &pixel_projected);
		residual[0] = T(pixel_value) - pixel_projected;

		return true;
	}

	static ceres::CostFunction *Create(Eigen::Vector3d pl_, float pixel_value_, vk::PinholeCamera *pinhole_cam_, ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 1>> &interpolator_)
	{
		return (new ceres::AutoDiffCostFunction<
				PhotometricFactorPoseOnly, 1, 4, 3>(
			new PhotometricFactorPoseOnly(pl_, pixel_value_, pinhole_cam_, interpolator_)));
	}

	template <typename T>
	const bool isInImage(T u, T v) const
	{
		return (u - T(2) > T(0) && u + T(2) < T(pinhole_cam->width()) &&
				v - T(2) > T(0) && v - T(2) < T(pinhole_cam->height()));
	}

	Eigen::Vector3d pl; //上一帧图像的相空间坐标
	float pixel_value;	//上一帧图像的相空间坐标对应的像素值
	vk::PinholeCamera *pinhole_cam;
	// cv::Mat src_img;                     //当前帧图像
	const ceres::BiCubicInterpolator<ceres::Grid2D<uchar, 1>> &interpolator;
};

//没有深度的误差
struct EssentialErrorPoseOnly
{
	EssentialErrorPoseOnly(Eigen::Vector2d r_uv, Eigen::Vector2d c_uv)
		: ref_uv(Eigen::Vector3d(r_uv[0], r_uv[1], 1)), cur_uv(Eigen::Vector3d(c_uv[0], c_uv[1], 1))
	{
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_curr_ref{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_curr_ref{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 3> t_head;
		t_head << T(0), T(-t[2]), T(t[1]),
			T(t[2]), T(0), T(-t[0]),
			T(-t[1]), T(t(0)), 0;

		Eigen::Matrix<T, 3, 3> t_head_r = t_head * q_curr_ref.toRotationMatrix();

		Eigen::Matrix<T, 3, 1> ref_uv_{T(ref_uv[0]), T(ref_uv[1]), T(ref_uv[2])};
		Eigen::Matrix<T, 3, 1> cur_uv_{T(cur_uv[0]), T(cur_uv[1]), T(cur_uv[2])};

		residual[0] = ref_uv_.transpose() * t_head_r * cur_uv_;
	}

	Eigen::Vector3d ref_uv;
	Eigen::Vector3d cur_uv;
};

//线段投影误差（两个端点到对应线段的距离）
struct LineSegmentFactorPoseOnly
{
	LineSegmentFactorPoseOnly(Eigen::Vector3d ps_, Eigen::Vector3d pe_, Eigen::Vector3d obs_line_, vk::PinholeCamera *pinhole_cam_)
		: ps(ps_), pe(pe_), obs_line(obs_line_), pinhole_cam(pinhole_cam_)
	{
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_curr_last{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_curr_last{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> ps_{T(ps[0]), T(ps[1]), T(ps[2])};
		Eigen::Matrix<T, 3, 1> ps_c = q_curr_last * ps_ + t_curr_last;

		Eigen::Matrix<T, 3, 1> pe_{T(pe[0]), T(pe[1]), T(pe[2])};
		Eigen::Matrix<T, 3, 1> pe_c = q_curr_last * pe_ + t_curr_last;

		if (ps_c.z() <= T(0) || pe_c.z() <= T(0))
			return false;

		T su = ps_c[0] / ps_c[2] * T(pinhole_cam->fx()) + T(pinhole_cam->cx());
		T sv = ps_c[1] / ps_c[2] * T(pinhole_cam->fy()) + T(pinhole_cam->cy());

		T eu = pe_c[0] / pe_c[2] * T(pinhole_cam->fx()) + T(pinhole_cam->cx());
		T ev = pe_c[1] / pe_c[2] * T(pinhole_cam->fy()) + T(pinhole_cam->cy());

		//线段起始点到线段的距离
		residual[0] = su * T(obs_line[0]) + sv * T(obs_line[1]) + T(obs_line[2]);

		//线段终点到线段的距离
		residual[1] = eu * T(obs_line[0]) + ev * T(obs_line[1]) + T(obs_line[2]);

		// residual[0] = err.norm() /;

		return true;
	}

	Eigen::Vector3d ps;		  // 上一帧图像线段起始点的相空间坐标
	Eigen::Vector3d pe;		  // 上一帧图像线段起终点的相空间坐标
	Eigen::Vector3d obs_line; // 当前帧对应的线段的直线方程
	vk::PinholeCamera *pinhole_cam;
};

//视觉光束法平差误差函数
struct SnavelyReprojectionFactor
{
	SnavelyReprojectionFactor(double observed_x, double observed_y, vk::PinholeCamera *pinhole_cam)
		: observed_x(observed_x), observed_y(observed_y), mpPinhole_cam(pinhole_cam) {}

	template <typename T>
	bool operator()(const T *q, const T *t,
					const T *const point,
					T *residuals) const
	{
		Eigen::Quaternion<T> q_curr_w{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_curr_w{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> pw{point[0], point[1], point[2]};
		Eigen::Matrix<T, 3, 1> pc = q_curr_w * pw + t_curr_w;

		if (pc.z() <= T(0))
			return false;

		T predicted_x = pc[0] / pc[2] * T(mpPinhole_cam->fx()) + T(mpPinhole_cam->cx());
		T predicted_y = pc[1] / pc[2] * T(mpPinhole_cam->fy()) + T(mpPinhole_cam->cy());

		// The error is the difference between the predicted and observed position.
		residuals[0] = T(observed_x) - predicted_x;
		residuals[1] = T(observed_y) - predicted_y;
		return true;
	}

	// Factory to hide the construction of the CostFunction object from
	// the client code.
	static ceres::CostFunction *Create(const double observed_x,
									   const double observed_y,
									   vk::PinholeCamera *pinhole_cam)
	{
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionFactor, 2, 4, 3, 3>(
			new SnavelyReprojectionFactor(observed_x, observed_y, pinhole_cam)));
	}

	double observed_x;
	double observed_y;
	vk::PinholeCamera *mpPinhole_cam;
};



class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
	EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_)
	{
	}
	virtual ~EdgeAnalyticCostFunction() {}
	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
	{
		Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
		Eigen::Vector3d lp;
		lp = q_last_curr * curr_point + t_last_curr;

		Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
		Eigen::Vector3d de = last_point_a - last_point_b;
		double de_norm = de.norm();
		residuals[0] = nu.norm() / de_norm;

		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Matrix3d skew_lp = skew(lp);
				Eigen::Matrix<double, 3, 6> dp_by_se3;
				dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
				(dp_by_se3.block<3, 3>(0, 3)).setIdentity();
				Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
				J_se3.setZero();
				Eigen::Matrix3d skew_de = skew(de);
				J_se3.block<1, 6>(0, 0) = -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
			}
		}
		return true;
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d last_point_a;
	Eigen::Vector3d last_point_b;
};



class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7>
{
public:
	SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_)
		: curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_)
	{
	}

	virtual ~SurfNormAnalyticCostFunction() {}
	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
	{
		Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
		Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;
		residuals[0] = plane_unit_norm.dot(point_w) + negative_OA_dot_norm;

		if (jacobians != NULL)
		{
			if (jacobians[0] != NULL)
			{
				Eigen::Matrix3d skew_point_w = skew(point_w);
				Eigen::Matrix<double, 3, 6> dp_by_se3;
				dp_by_se3.block<3, 3>(0, 0) = -skew_point_w;
				(dp_by_se3.block<3, 3>(0, 3)).setIdentity();
				Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
				J_se3.setZero();
				J_se3.block<1, 6>(0, 0) = plane_unit_norm.transpose() * dp_by_se3;
			}
		}
		return true;
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};




class PoseSE3Parameterization : public ceres::LocalParameterization
{
public:
	PoseSE3Parameterization() {}
	virtual ~PoseSE3Parameterization() {}
	virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
	{
		Eigen::Map<const Eigen::Vector3d> trans(x + 4);

		Eigen::Quaterniond delta_q;
		Eigen::Vector3d delta_t;
		getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(delta), delta_q, delta_t);
		Eigen::Map<const Eigen::Quaterniond> quater(x);
		Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
		Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

		quater_plus = delta_q * quater;
		trans_plus = delta_q * trans + delta_t;

		return true;
	}
	virtual bool ComputeJacobian(const double *x, double *jacobian) const
	{
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
		(j.topRows(6)).setIdentity();
		(j.bottomRows(1)).setZero();

		return true;
	}
	virtual int GlobalSize() const { return 7; }
	virtual int LocalSize() const { return 6; }
};



#ifdef USE_MARKER

//只优化当前帧的pose，Marker的空间位姿固定
struct MarkerReprojectionOnlyPose
{
	MarkerReprojectionOnlyPose(Sophus::SE3 posMarker, aruco::Marker &marker, vk::PinholeCamera *pCameraModel)
		: posMarker_(posMarker), marker_(marker), pCameraModel_(pCameraModel)
	{
		d_[0] = pCameraModel_->d0();
		d_[1] = pCameraModel_->d1();
		d_[2] = pCameraModel_->d2();
		d_[3] = pCameraModel_->d3();
		d_[4] = pCameraModel_->d4();

		fx_ = pCameraModel_->fx();
		fy_ = pCameraModel_->fy();
		cx_ = pCameraModel_->cx();
		cy_ = pCameraModel_->cy();
	}

	template <typename T>
	Eigen::Matrix<T, 2, 1> project2d(Eigen::Matrix<T, 3, 1> &pc) const
	{
		Eigen::Matrix<T, 2, 1> pc_norm;
		pc_norm[0] = pc[0] / pc[2];
		pc_norm[1] = pc[1] / pc[2];
		return pc_norm;
	}

	template <typename T>
	Eigen::Matrix<T, 2, 1> world2cam(Eigen::Matrix<T, 3, 1> &pc) const
	{
		Eigen::Matrix<T, 2, 1> pc_norm = project2d(pc);
		return world2cam(pc_norm);
	}

	template <typename T>
	Eigen::Matrix<T, 2, 1> world2cam(Eigen::Matrix<T, 2, 1> &pc_unit) const
	{
		Eigen::Matrix<T, 2, 1> px;
		if (fabs(d_[0]) < 0.0001)
		{
			px[0] = T(fx_) * pc_unit[0] + T(cx_);
			px[1] = T(fy_) * pc_unit[1] + T(cy_);
		}
		else
		{
			T x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
			x = pc_unit[0];
			y = pc_unit[1];
			r2 = x * x + y * y;
			r4 = r2 * r2;
			r6 = r4 * r2;
			a1 = T(2) * x * y;
			a2 = r2 + T(2) * x * x;
			a3 = r2 + T(2) * y * y;
			cdist = T(1) + T(d_[0]) * r2 + T(d_[1]) * r4 + T(d_[4]) * r6;
			xd = x * cdist + T(d_[2]) * a1 + T(d_[3]) * a2;
			yd = y * cdist + T(d_[2]) * a3 + T(d_[3]) * a1;
			px[0] = xd * T(fx_) + T(cx_);
			px[1] = yd * T(fy_) + T(cy_);
		}
		return px;
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		//相机的姿态Tcw
		Eigen::Quaternion<T> q_cam_world{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_cam_world{t[0], t[1], t[2]};

		// //Marker的世界坐标系的姿态
		Sophus::SE3 Twm = posMarker_;

		//将世界坐标系中的marker投影到当前帧图像像空间坐标
		float marker_size = marker_.ssize;
		float half_marker_size = marker_size / 2;
		Eigen::Vector3d marker_corner_1(-half_marker_size, half_marker_size, 0);
		Eigen::Vector3d marker_corner_2(half_marker_size, half_marker_size, 0);
		Eigen::Vector3d marker_corner_3(half_marker_size, -half_marker_size, 0);
		Eigen::Vector3d marker_corner_4(-half_marker_size, -half_marker_size, 0);

		Eigen::Vector3d mw_1 = Twm * marker_corner_1;
		Eigen::Vector3d mw_2 = Twm * marker_corner_2;
		Eigen::Vector3d mw_3 = Twm * marker_corner_3;
		Eigen::Vector3d mw_4 = Twm * marker_corner_4;

		//将角点投影到图像坐标系
		Eigen::Matrix<T, 3, 1> p_cam1 = q_cam_world * mw_1.cast<T>() + t_cam_world;
		Eigen::Matrix<T, 2, 1> uv1 = world2cam<T>(p_cam1);

		residual[0] = T(marker_[0].x) - uv1[0];
		residual[1] = T(marker_[0].y) - uv1[1];

		Eigen::Matrix<T, 3, 1> p_cam2 = q_cam_world * mw_2.cast<T>() + t_cam_world;
		Eigen::Matrix<T, 2, 1> uv2 = world2cam<T>(p_cam2);
		residual[2] = T(marker_[1].x) - uv2[0];
		residual[3] = T(marker_[1].y) - uv2[1];

		Eigen::Matrix<T, 3, 1> p_cam3 = q_cam_world * mw_3.cast<T>() + t_cam_world;
		Eigen::Matrix<T, 2, 1> uv3 = world2cam<T>(p_cam3);
		residual[4] = T(marker_[2].x) - uv3[0];
		residual[5] = T(marker_[2].y) - uv3[1];

		Eigen::Matrix<T, 3, 1> p_cam4 = q_cam_world * mw_4.cast<T>() + t_cam_world;
		Eigen::Matrix<T, 2, 1> uv4 = world2cam<T>(p_cam4);
		residual[6] = T(marker_[3].x) - uv4[0];
		residual[7] = T(marker_[3].y) - uv4[1];

		return true;
	}

	static ceres::CostFunction *Create(Sophus::SE3 posMarker, aruco::Marker &marker, vk::PinholeCamera *pCameraModel)
	{
		//误差维度，每个参数块维度
		return (new ceres::AutoDiffCostFunction<
				MarkerReprojectionOnlyPose, 8, 4, 3>(
			new MarkerReprojectionOnlyPose(posMarker, marker, pCameraModel)));
	}

	vk::PinholeCamera *pCameraModel_;
	aruco::Marker marker_;
	Sophus::SE3 posMarker_;

	double d_[5] = {0};
	double fx_, fy_, cx_, cy_;
};

#endif //USE_MARKER

// 代价函数的计算模型
struct CURVE_FITTING_COST
{
	CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
	// 残差的计算
	template <typename T>
	bool operator()(
		const T *const abc, // 模型参数，有3维
		T *residual) const	// 残差
	{
		residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
		return true;
	}
	const double _x, _y; // x,y数据
};

class CurveFittingCostFunctor
{
public:
	CurveFittingCostFunctor(double x, double y)
		: _x(x), _y(y)
	{
	}

	// 残差的计算
	template <typename T>
	bool operator()(
		const T *const abc, // 模型参数，有3维
		T *residual) const	// 残差
	{
		residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
		return true;
	}

	static ceres::CostFunction *Create(const double x, const double y)
	{
		return (new ceres::AutoDiffCostFunction<CurveFittingCostFunctor, 1, 3>(
			new CurveFittingCostFunctor(x, y)));
	}

private:
	double _x, _y;
};



#endif//CERES_FACTOR_H