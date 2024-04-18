#pragma once
#define FMT_HEADER_ONLY
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "fmt/format.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

// 数据类型定义
using SO3 = Sophus::SO3d;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec18 = Eigen::Matrix<double, 18, 1>;
using Mat18 = Eigen::Matrix<double, 18, 18>;