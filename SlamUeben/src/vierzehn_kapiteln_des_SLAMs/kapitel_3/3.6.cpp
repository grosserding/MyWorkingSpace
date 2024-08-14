#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>

int main(int argc, char** argv) {
  // rotation matrix
  Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();  // 初始化为单位矩阵
  // rotation vector
  Eigen::AngleAxisd rot_vec(M_PI / 4,
                            Eigen::Vector3d(0, 0, 1));  // 沿z轴旋转45度
  std::cerr.precision(3);                               // 控制打印精度
  std::cerr << "rot_mat = \n" << rot_mat << std::endl;
  Eigen::AngleAxisd rot_vec_0(0, Eigen::Vector3d::Zero());
  std::cerr << "rot_vec_0.matrix() = \n" << rot_vec_0.matrix() << std::endl;
  std::cerr << "rot_vec.matrix() = \n"
            << rot_vec.matrix() << std::endl;  // 旋转向量.matrix()
  std::cerr << "rot_vec.toRotationMatrix() = \n"
            << rot_vec.toRotationMatrix() << std::endl;  // 旋转向量.matrix()

  // use rotation vector or rotation matrix to transform
  Eigen::Vector3d pos(1, 0, 0);
  auto pos1 = rot_vec * pos;
  auto pos2 = rot_vec.toRotationMatrix() * pos;
  std::cerr << "pos1 = " << pos1.transpose() << std::endl;
  std::cerr << "pos2 = " << pos2.transpose() << std::endl;

  // Use Euler angle
  auto RPY = rot_vec.toRotationMatrix().eulerAngles(0, 1, 2);
  std::cerr << "RPY = " << RPY.transpose() << std::endl;
  std::cerr << "typeid(RPY).name() = " << typeid(RPY).name() << std::endl;

  // euler-transformation using Eigen::Isometry
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  std::cerr << "Isometry3d.matrix() is like \n" << T.matrix() << std::endl;
  // Eigen::Isometry3d is actually 4x4
  T.rotate(rot_vec);
  T.pretranslate(Eigen::Vector3d(1, 2, 3));
  std::cerr << "Isometry3d.matrix() after setting value is \n"
            << T.matrix() << std::endl;
  // it is exactly the same with SE3d.matrix()

  Eigen::Vector3d vec1(0, 1, 2);
  auto vec2 = T * vec1;  // same with vec2 = R * vec1 + t;
  std::cerr << "vec2 = " << vec2.transpose() << std::endl;

  // ****** Uebungen ******* //
  {
    // Uebung 5
    // 矩阵可以这样被随机赋值，随机值在-1~1之间，::Random();
    Eigen::Matrix<double, 8, 8> mat_test =
        Eigen::Matrix<double, 8, 8>::Random();
    mat_test.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    std::cerr << "mat_test = \n" << mat_test << std::endl;
    // Uebung 7
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
    std::cerr << "q1.coeff = " << q1.coeffs() << std::endl;
    q1.normalize();  // 归一化的函数为 normalize();
    std::cerr << "q1.coeff after normalize = " << q1.coeffs() << std::endl;
    Eigen::Vector3d t1(0.3, 0.1, 0.1);
    Eigen::Isometry3d T1;
    T1.rotate(q1);
    T1.pretranslate(t1);
    Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
    q2.normalize();
    Eigen::Vector3d t2(-0.1, 0.5, 0.3);
    Eigen::Isometry3d T2;
    T2.rotate(q2);
    T2.pretranslate(t2);
    auto pos1 = Eigen::Vector3d(0.5, 0, 0.2);
    auto pos2 = T2.inverse() * T1 * pos1;
    std::cerr << "pos1 = " << pos1.transpose() << std::endl;
    std::cerr << "pos2 = " << pos2.transpose() << std::endl;
  }

  return 0;
}