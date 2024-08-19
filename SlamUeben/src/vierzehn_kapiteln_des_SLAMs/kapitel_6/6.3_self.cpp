#include <ceres/ceres.h>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <opencv2/core/core.hpp>

// fitting y = Rx + t
struct TRANSFORM_FITTING_COST {
  TRANSFORM_FITTING_COST(Eigen::Vector3d x, Eigen::Vector3d y) : _x(x), _y(y) {}
  // calculate residual
  //   template <typename T>
  //   bool operator()(const T* const abc,  // 模型参数，有3维
  //                   T* residual) const   // 残差
  //   {
  //     residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] *
  //     T(_x) +
  //                                      abc[2]);  // y-exp(ax^2+bx+c)
  //     return true;
  //   }
  bool operator()(const Eigen::Matrix4d* const R_t,
                  double* residual) const {  // 函数调用运算符 的重载
    // abc 为模型参数，有3维
    residual[0] =
        _y - (R_t->block<3, 3>(0, 0) * _x + R_t->block<3, 1>(0, 3) * _y);
    return true;
  }
  const Eigen::Vector3d _x, _y;
};

int main(int argc, char** argv) {
  // build a least square problem: y = R x + t, y: 3x1, R: 3x3, x: 3x1, t: 3x1
  std::cerr << "generating R_t fitting problem data" << std::endl;
  Eigen::AngleAxisd rot_vec(M_PI / 2, Eigen::Vector3d(0, 0, 1));
  Eigen::Matrix3d R_real = rot_vec.toRotationMatrix();
  Eigen::Vector3d t_real(0, 0, -1);
  int N = 100;
  double w_sigma = 1.0;
  cv::RNG rng;
  Eigen::Matrix4d T = Eigen::Matrix4d::Zero();

  std::vector<Eigen::Vector3d> x_data, y_data;

  std::cerr << "generating data: " << std::endl;
  for (int i = 0; i < N; i++) {
    Eigen::Vector3d x(i / 10, i / 10, i / 10);
    Eigen::Vector3d y =
        R_real * x + t_real + Eigen::Vector3d(rng.gaussian(w_sigma));
    x_data.emplace_back(x);
    y_data.emplace_back(y);
  }

  ceres::Problem problem;
  //   for(int i = 0; i < N; i++) {
  //     problem.AddResidualBlock(
  //         new ceres::AutoDiffCostFunction<TRANSFORM_FITTING_COST
  //     )
  return 0;
}