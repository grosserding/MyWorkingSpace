#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>
#include <string>

namespace transform {

template <typename S>
Eigen::Matrix<S,3,1> ToRollPitchYaw(const Eigen::Quaternion<S>& rotation) {
  const S& w = rotation.w();
  const S& x = rotation.x();
  const S& y = rotation.y();
  const S& z = rotation.z();

  S sinr_cosp = +2.0 * (w * x + y * z);
  S cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
  S roll = atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  S sinp = +2.0 * (w * y - z * x);
  S pitch;
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);
  // yaw (z-axis rotation)
  S siny_cosp = +2.0 * (w * z + x * y);
  S cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
  S yaw = atan2(siny_cosp, cosy_cosp);
  return Eigen::Matrix<S,3,1>(roll, pitch, yaw);
}

double DegreeToRad(const double& degree) { return degree * M_PI / 180.0; }

double RadToDegree(const double& rad) { return rad * 180.0 / M_PI; }

Eigen::Quaterniond RollPitchYaw(const double& roll, const double& pitch,
                                const double& yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

Sophus::SE3d Embed3D(const Sophus::SE2d& se2) {
  Sophus::SE3d se3;
  se3.translation() =
      Eigen::Vector3d(se2.translation()(0), se2.translation()(1), 0);
  se3.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, se2.so2().log()));
  return se3;
}

Sophus::SE2d Project2D(const Sophus::SE3d& se3) {
  Sophus::SE2d se2;
  se2.translation() =
      Eigen::Vector2d(se3.translation()(0), se3.translation()(1));
  se2.so2() = Sophus::SO2d::exp(se3.so3().log()[2]);
  return se2;
}

std::string DebugString(const Sophus::SE3d& se3) {
  std::ostringstream ss;
  Eigen::Vector3d rpy = ToRollPitchYaw(se3.so3().unit_quaternion());
  ss << "{ t:[" << se3.translation().x() << ", " << se3.translation().y()
     << ", " << se3.translation().z() << "]"
     << ", r:[" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "]}.";
  return ss.str();
}

std::string DebugString(const Sophus::SE2d& se2) {
  std::ostringstream ss;
  ss << "{ t:[" << se2.translation().x() << ", " << se2.translation().y() << "]"
     << ", r:[" << se2.so2().log() << "]}.";
  return ss.str();
}

double GetYaw(const Sophus::SE2d& se2) { return se2.so2().log(); }

double GetYaw(const Eigen::Quaterniond& rotation) {
  Eigen::Vector3d direction = rotation * Eigen::Vector3d::UnitX();
  return atan2(direction.y(), direction.x());
}

Eigen::Vector4d EstiPlane(const std::vector<Eigen::Vector3d>& points) {
  Eigen::Matrix<double, 4, 3> A;
  Eigen::Matrix<double, 4, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;
  for (int j = 0; j < 4; j++) {
    Eigen::Vector3d tmp_point = points[j];
    A(j, 0) = tmp_point(0);
    A(j, 1) = tmp_point(1);
    A(j, 2) = tmp_point(2);
  }
  // Ax = b
  Eigen::Matrix<double, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
  double n = normvec.norm();
  Eigen::Vector4d pca_result;
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;
  return pca_result;
}


}  // namespace transform
