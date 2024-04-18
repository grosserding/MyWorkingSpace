#pragma once
#include "datatype/datatype.h"
#include "sensors/imu.h"
#include <cmath>
#include <iostream>
#include <typeinfo>

namespace EKFUeben {
class Filter {
public:
  void RunFilter();

private:
};

// template <typename S> 这里可以为了提高通用性设置模板类
// template <typename S = double> 模板类也可以设置默认数据类型
struct NavState {

  NavState() = default;
  // timestamp, p, v, q, ba, bg
  double timestamp_ = 0;
  SO3 R_ = SO3(Eigen::Matrix3d::Identity());
  Vec3 p_ = Vec3::Zero();
  Vec3 v_ = Vec3::Zero();
  Vec3 bg_ = Vec3::Zero();
  Vec3 ba_ = Vec3::Zero();
  Vec3 g_{0, 0, -9.8};
};

class ESKF {
public:
  bool Predict(const sensors::IMU imu);

private:
  // 名义状态，可以看到名义状态是分开定义的，比如旋转定义为SO3
  Vec3 p_ = Vec3::Zero();
  Vec3 v_ = Vec3::Zero();
  SO3 R = SO3(Mat3::Identity());
  Vec3 ba_ = Vec3::Zero();
  Vec3 bg_ = Vec3::Zero();

  // 误差状态则被直接定义为一整个18的vector
  Vec18 dx_ = Vec18::Zero();

  // 协方差阵
  Mat18 cov_ = Mat18::Identity();
};
} // namespace EKFUeben
