#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

int main(int argc, char** argv) {
  std::cerr << "this is 14_3_2" << std::endl;
  Eigen::Matrix<double, 2, 3> matrix_23;
  Eigen::Vector3d v_3d;
  Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();  // Zero init
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
      matrix_dynamic;  // 动态大小矩阵
  Eigen::MatrixXd matrix_x;

  matrix_23 << 1, 2, 3, 4, 5, 6;  // matrix 赋值
  std::cerr << "matrix_23 = \n" << matrix_23 << std::endl;

  for (int i = 0; i < 2; i++) { // 根据行列索引访问矩阵元素
    for (int j = 0; j < 3; j++) {
      std::cerr << "matrix(" << i << ", " << j << ") = " << matrix_23(i, j)
                << std::endl;
    }
  }
  v_3d << 3, 2, 1;

  Eigen::Matrix<float, 2, 1> matrix_21 =
      matrix_23.cast<float>() * v_3d.cast<float>(); // 转换数据类型用 .cast<type>()
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 1; j++) {
      std::cerr << "matrix(" << i << ", " << j << ") = " << matrix_21(i, j)
                << std::endl;
    }
  }
  return 1;
}