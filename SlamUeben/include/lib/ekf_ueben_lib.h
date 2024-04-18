#pragma once
#define FMT_HEADER_ONLY
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <typeinfo>

#include "fmt/format.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
namespace EKFUeben {
class Filter {
 public:
    void RunFilter();
 private:
};
}  // namespace EKFUeben
