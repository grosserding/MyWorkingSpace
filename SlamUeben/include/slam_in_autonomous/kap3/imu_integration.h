#pragma once
#include "common/common_include.h"

struct IMU {
  IMU() = default;
  IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc)
      : _stamp(t), _gyro(gyro), _acc(acc) {}
  double _stamp = 0.0;
  Eigen::Vector3d _gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d _acc = Eigen::Vector3d::Zero();
};

class IMUIntegration {
 public:
  IMUIntegration(const Eigen::Vector3d& gravity, const Eigen::Vector3d& init_bg,
                 const Eigen::Vector3d& init_ba)
      : _gravity(gravity), _bg(init_bg), _ba(init_ba) {}
  IMUIntegration()
      : _gravity(Eigen::Vector3d(0, 0, 0.98)),
        _bg(Eigen::Vector3d::Zero()),
        _ba(Eigen::Vector3d::Zero()) {}

  void AddIMU(const IMU& imu) {
    if (!_stamp) {
      _stamp = imu._stamp;
      return;
    }
    double dt = imu._stamp - _stamp;
    // 先利用当前的旋转矩阵计算速度和平移量，然后再计算旋转矩阵
    _v += (_R * (imu._acc - _ba) + _gravity) * dt;
    _p += _v * dt + 0.5 * (_R * (imu._acc - _ba) + _gravity) * dt * dt;
    Sophus::SO3d dR = Sophus::SO3d::exp((imu._gyro - _bg) * dt);
    _R = _R * dR;
    // test ，看一下SO3里面左乘还是右乘的影响
    std::cout << "_R * dR = \n" << (_R * dR).matrix() << std::endl;
    std::cout << " dR * _R= \n" << (dR * _R).matrix() << std::endl;
    _stamp = imu._stamp;
  }

  Sophus::SO3d GetR() const { return _R; }
  Eigen::Vector3d GetV() const { return _v; }
  Eigen::Vector3d GetP() const { return _p; }

 private:
  // 时间量
  double _stamp = 0.0;

  // 状态量
  Sophus::SO3d _R = Sophus::SO3d(Eigen::Matrix3d::Identity());
  Eigen::Vector3d _p = Eigen::Vector3d::Zero();
  Eigen::Vector3d _v = Eigen::Vector3d::Zero();

  // 零偏量
  Eigen::Vector3d _bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d _ba = Eigen::Vector3d::Zero();

  // 重力量
  Eigen::Vector3d _gravity = Eigen::Vector3d::Zero();
};