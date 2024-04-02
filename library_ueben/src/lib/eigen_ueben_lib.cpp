#include "lib/eigen_ueben_lib.h"

namespace HelloWorld {
void PrintHelloWorld() {
  std::cout << "library ueben!" << std::endl;
  std::cerr << "new line in new Ubuntu 22.04" << std::endl;
}
}  // namespace HelloWorld

namespace EigenLibs {
void EigenUebungs() {
  //****** Basics ******//
  std::cerr << "****** Basics ******" << std::endl;
  Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
  std::cerr << "Eigen::Matrix3d::Identity:\n"
            << Eigen::Matrix3d::Identity() << std::endl;

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
  
}
}  // namespace EigenLibs