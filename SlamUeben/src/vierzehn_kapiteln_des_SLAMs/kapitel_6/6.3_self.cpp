#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <ceres/ceres.h>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sophus/common.hpp>
#include <sophus/se3.hpp>

int main(int argc, char** argv) {
  // 已知的向量 a 和 b
  Eigen::Vector3d a(36.682861 + 107.339447, 444.794983 - 368.235229,
                    -2.003768 - 7.203767);  // 你可以根据需要更改向量的值
  Eigen::Vector3d b(35.070869 + 106.496429, 450.609863 - 369.124573,
                    7.601254 - 6.662053);  // 你可以根据需要更改向量的值

  // 规范化向量，消除长度误差的影响
  Eigen::Vector3d a_norm = a.normalized();
  Eigen::Vector3d b_norm = b.normalized();

  // 计算旋转矩阵
  Eigen::Matrix3d rotation_matrix =
      Eigen::Quaterniond().setFromTwoVectors(a_norm, b_norm).toRotationMatrix();

  // 计算平移向量
  Eigen::Vector3d translation_vector = b - rotation_matrix * a;

  // 使用 Sophus 表示欧氏变换
  Sophus::SE3d transformation(rotation_matrix, translation_vector);

  // 输出结果
  std::cout << "Rotation Matrix: \n" << rotation_matrix << std::endl;
  std::cout << "Translation Vector: \n"
            << translation_vector.transpose() << std::endl;

  auto b_test = transformation * a;
  std::cerr << "b_test = " << b_test.transpose() << std::endl;
  std::cerr << "a = " << a.transpose() << std::endl;
  std::cerr << "b = " << b.transpose() << std::endl;
  // std::cout << "SE3 Transformation: \n" << transformation.matrix() <<
  // std::endl;
}