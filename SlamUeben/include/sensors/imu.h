#pragma once
#include "datatype/datatype.h"
#include <memory>

namespace sensors {
struct IMU {
  IMU() = default;
  IMU(double t, const Vec3 &gyro, const Vec3 &acc) : gyro_(gyro), acc_(acc) {}
  Vec3 gyro_ = Vec3::Zero();
  Vec3 acc_ = Vec3::Zero();
};

} // namespace sensors