#include "lib/eigen_ueben_lib.h"
#define MATRIX_SIZE 100
namespace HelloWorld {
void PrintHelloWorld() {
  std::cout << "library ueben!" << std::endl;
  std::cerr << "new line in new Ubuntu 22.04" << std::endl;
}
} // namespace HelloWorld

namespace EigenLibs {
void EigenUebungs() {
  //****** Basics ******//
  std::cerr << "****** Basics ******" << std::endl;
  Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
  std::cerr << "Eigen::Matrix3d::Identity:\n"
            << Eigen::Matrix3d::Identity() << std::endl;

  Eigen::Vector3d vec1(0, 1, 2);
  std::cerr << "type of vec1 is: " << typeid(vec1(0)).name()
            << ", value is: " << vec1(0) << std::endl;
  auto vec2 = vec1.cast<float>();
  std::cerr << "type of vec2 is: " << typeid(vec2(0)).name()
            << ", value is: " << vec2(0) << std::endl;
  //****** Rotation Matrix - Quaternion - EulerAngle ******//

  std::cerr << "****** Rotation Matrix - Quaternion - EulerAngle ******"
            << std::endl;
  std::cerr << "EulerAngle -> RotationMatrix: " << std::endl;
  std::cerr << "EulerAngle = 15°, 15°, 45°;" << std::endl;
  auto roll = Eigen::AngleAxisd(15 / 180 * PI, Eigen::Vector3d::UnitX());
  auto pitch = Eigen::AngleAxisd(15 / 180 * PI, Eigen::Vector3d::UnitY());
  auto yaw = Eigen::AngleAxisd(45 / 180 * PI, Eigen::Vector3d::UnitZ());
  auto rot = roll * pitch * yaw;
  std::cerr << " roll * pitch * yaw的类型为: " << typeid(rot).name()
            << std::endl;
  std::cerr << " roll * pitch的类型为: " << typeid(roll * pitch).name()
            << std::endl;
  // 因此，AngleAxis相乘得到的数据类型为Quaternion

  //****** SLAM十四讲习题 ******//
  // 3.4 旋转矩阵/轴角/欧拉角/四元数之间的转化关系
  Eigen::Matrix3d rot_1;
  Eigen::AngleAxisd aa;
  Eigen::Vector3d rpy;
  Eigen::Quaterniond quat;
  rpy = Eigen::Vector3d(0, 0, PI / 4);
  aa = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
  quat = Eigen::Quaterniond(aa);
  rot_1 = Eigen::Matrix3d(quat);
  // 注意，AngleAxis不能直接被打印
  // std::cerr << "aa = " << aa << ", quat = " << quat << ", rot = " << rot_1
  //           << std::endl;
  // std::cerr << "quat = " << quat << std::endl << "rot = " << rot_1 << std::endl;
  std::cerr << "Eigen::Vector3d::UnitZ() 的类型是什么:\n"
            << typeid(Eigen::Vector3d::UnitZ()).name()
            << "Eigen::Vector3d::UnitZ() 打印出来是什么:\n"
            << Eigen::Vector3d::UnitZ() << std::endl;
  // 注意：Eigen::Vector3d::UnitX() 本质也是一个Vector3d!
  // 3.5 取左上角3*3的矩阵快，然后赋值为I
  Eigen::MatrixXd mat_big = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  std::cerr << "mat_big 3*3 origin = \n"
            << mat_big.block<3, 3>(0, 0) << std::endl;
  mat_big.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  std::cerr << "mat_big 3*3 after = \n"
            << mat_big.block<3, 3>(0, 0) << std::endl;
  std::cerr << "random 3*3 matrix = \n"
            << Eigen::Matrix3d::Random() << std::endl;
  // 注意：MatrixX代表尺寸完全未知的矩阵，而非方阵！故调用时如果要指定尺寸需要同时指定横纵向
  // 3.6 Ax = b的做法？
  Eigen::Matrix3d A = Eigen::Matrix3d::Random();
  auto b = Eigen::Vector3d::Random();
  auto x = A.inverse() * b;
  std::cerr << "x = A.inverse() * b =\n" << x << std::endl;
  auto x_qr = A.householderQr().solve(b);
  std::cerr << "x = A.householderQr().solve(b) =\n" << x_qr << std::endl;
  // 3.7 坐标转换
  auto pose1 = Sophus::SE3d();
}
} // namespace EigenLibs