#pragma once
#include <typeinfo>
#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include "common/csv_utils.hpp"
#include "common/kbhit.h"
#include "common/singleton.hpp"
#include "common/transform.h"
#include "fmt/format.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"