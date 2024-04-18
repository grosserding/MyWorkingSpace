#include "lib/libraries_ueben_lib.h"
#define MATRIX_SIZE 100
namespace HelloWorld {
void PrintHelloWorld() {
  std::cout << "library ueben!" << std::endl;
  std::cerr << "new line in new Ubuntu 22.04" << std::endl;
}
} // namespace HelloWorld

template <typename S>
Eigen::Matrix<S, 3, 1> ToRollPitchYaw(const Eigen::Quaternion<S> &rotation) {
  const S &w = rotation.w();
  const S &x = rotation.x();
  const S &y = rotation.y();
  const S &z = rotation.z();

  S sinr_cosp = +2.0 * (w * x + y * z);
  S cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
  S roll = atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  S sinp = +2.0 * (w * y - z * x);
  S pitch;
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);
  // yaw (z-axis rotation)
  S siny_cosp = +2.0 * (w * z + x * y);
  S cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
  S yaw = atan2(siny_cosp, cosy_cosp);
  return Eigen::Matrix<S, 3, 1>(roll, pitch, yaw);
}

namespace LibrariesUeben {
void LibrarysUebenFunc() {
  //****** Basics ******//
  std::cerr << "****** Basics ******" << std::endl;
  {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
    std::cerr << "Eigen::Matrix3d::Identity:\n"
              << Eigen::Matrix3d::Identity() << std::endl;

    Eigen::Vector3d vec1(0, 1, 2);
    std::cerr << "type of vec1 is: " << typeid(vec1(0)).name()
              << ", value is: " << vec1(0) << std::endl;
    auto vec2 = vec1.cast<float>();
    std::cerr << "type of vec2 is: " << typeid(vec2(0)).name()
              << ", value is: " << vec2(0) << std::endl;
  }
  //****** Rotation Matrix - Quaternion - EulerAngle ******//
  {
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

  //****** 14chaps chap3 ******//
  {
    std::cerr << "****** 14chaps chap3 ******" << std::endl;
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
    // std::cerr << "quat = " << quat << std::endl << "rot = " << rot_1 <<
    // std::endl;
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

    // temp
    Eigen::Matrix<double, 9, 3> mask = Eigen::Matrix<double, 9, 3>::Zero();
    // mask(0, 6) = 1;
    mask(7, 1) = 1;
    std::cerr << "mask = " << mask << std::endl;
    Eigen::Matrix3d rot_mat;
    rot_mat << 0.99953, 0.0145026, -0.0270589, 0.0272467, -0.0129099, 0.999546,
        0.0141467, -0.999811, -0.0132991;
    Eigen::Vector3d R_ZYX = rot_mat.eulerAngles(2, 1, 0);
    std::cout << "yaw, pitch, roll " << R_ZYX.transpose() * 180 / M_PI
              << std::endl;

    Eigen::Quaterniond quat_2(rot_mat);
    auto rpy_2 = ToRollPitchYaw(quat_2);
    std::cout << "roll, pitch, yaw " << rpy_2.transpose() * 180 / M_PI
              << std::endl;
  }
  //****** SO3 R euler AngleAxis ******//
  {
    std::cerr << "****** SO3 R euler AngleAxis ******" << std::endl;
    std::cerr
        << "#1. AngleAxis, 要再次明确，旋转向量为一个方向的向量，与欧拉角不同"
        << std::endl;
    {
      std::cerr << "##1.1 AngleAxis Initiation" << std::endl;
      Eigen::AngleAxisd aa_1(M_PI / 2, Eigen::Vector3d(0, 0, 1));
      Eigen::AngleAxisd aa_2(M_PI / 2, Eigen::Vector3d::UnitZ());
      // std::cerr << "aa_1 = " << aa_1 << std::endl;
      // std::cerr << "aa_2 = " << aa_2 << std::endl;
      std::cerr << "##1.2 AngleAxis to RotationMatrix" << std::endl;
      Eigen::Matrix3d rm_1 = aa_1.matrix();
      Eigen::Matrix3d rm_2 = aa_2.toRotationMatrix();
      // 这两个都可以！
      std::cerr << "rm_1 = \n" << rm_1 << std::endl;
      std::cerr << "rm_2 = \n" << rm_2 << std::endl;
      std::cerr << "##1.3 AngleAxis to EulerAngle(没有直接方法)" << std::endl;
      Eigen::Vector3d ea_1 = aa_1.matrix().eulerAngles(0, 1, 2);
      Eigen::Vector3d ea_2 = aa_2.matrix().eulerAngles(0, 1, 2);
      std::cerr << "ea_1 = \n" << ea_1.transpose() << std::endl;
      std::cerr << "ea_2 = \n" << ea_2.transpose() << std::endl;
      std::cerr << "##1.4 AngleAxis to Quaternion" << std::endl;
      Eigen::Quaterniond quat_1(aa_1);
      Eigen::Quaterniond quat_2(aa_2);
      std::cerr << "quat_1 = \n"
                << quat_1.x() << "," << quat_1.y() << "," << quat_1.z() << ","
                << quat_1.w() << std::endl;
      // 注意这里打印四元数的方式，不能直接流输出，而是要调用coeffs()函数来返回Matrix
      std::cerr << "quat_2 = \n" << quat_2.coeffs().transpose() << std::endl;
      std::cerr
          << "#Summary: .matrix()=.toRotationMatrix(), fromRotationMatrix()"
          << std::endl;
    }

    std::cerr << "#2. RotationMatrix" << std::endl;
    {
      std::cerr << "##2.1 RotationMatrix Initiation" << std::endl;
      Eigen::Matrix3d rm_1 =
          Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).matrix();
      std::cerr << "rm_1 = \n" << rm_1 << std::endl;
      std::cerr << "##2.2 RotationMatrix to AngleAxis" << std::endl;
      Eigen::AngleAxisd aa_1(rm_1);
      Eigen::AngleAxisd aa_2;
      aa_2.fromRotationMatrix(rm_1);
      std::cerr << "##2.3 RotationMatrix to EulerAngles" << std::endl;
      Eigen::Vector3d ea = rm_1.eulerAngles(0, 1, 2);
      std::cerr << "rm_1.eulerAngles(0, 1, 2) = " << ea.transpose() / M_PI * 180
                << "(deg)" << std::endl;
      std::cerr << "##2.4 RotationMatrix to Quaterniond" << std::endl;
      std::cerr << "quat_1 = " << Eigen::Quaterniond(rm_1).coeffs()
                << std::endl;
    }
    std::cerr << "#3. EulerAngle" << std::endl;
    {
      std::cerr << "##3.1 EulerAngle Initiation" << std::endl;
      Eigen::Vector3d ea(M_PI / 4, M_PI / 4, M_PI / 4);
      std::cerr << "##3.2 EulerAngle to AngleAxis" << std::endl;
      Eigen::AngleAxisd aa_x(ea(0), Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd aa_y(ea(1), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd aa_z(ea(2), Eigen::Vector3d::UnitZ());
      auto aa_1 = aa_x * aa_y * aa_z;
      auto aa_4 = aa_z * aa_y * aa_x;
      std::cerr << "aa_1.type = " << typeid(aa_1).name() << std::endl;
      std::cerr << "aa_1.mat = " << aa_1.coeffs().transpose() << std::endl
                << "aa_4.mat = " << aa_4.coeffs().transpose() << std::endl;
      Eigen::Matrix3d rot_origin = Eigen::Matrix3d::Identity();
      auto rot_1 = aa_1 * rot_origin;
      auto rot_2 = aa_4 * rot_origin;
      std::cerr << "after aa_1 = " << rot_1 << std::endl;
      std::cerr << "after aa_4 = " << rot_2 << std::endl;
      // 注意：欧拉角三个轴的先后顺序对最后产生的结果有影响。
    }
    std::cerr << "#4. quaternion" << std::endl;
    {
      std::cerr << "##4.1 quaternion Initiation" << std::endl;
      Eigen::Quaterniond quat_1(1, 0.5, 0.5, 0.5);
      std::cerr << "##4.2 quaternion to RotationMatrix" << std::endl;
      Eigen::Matrix3d rm_1 = quat_1.matrix();
      Eigen::Matrix3d rm_2 = quat_1.toRotationMatrix();
      std::cerr << "rm_1 = \n" << rm_1 << std::endl;
      std::cerr << "rm_2 = \n" << rm_2 << std::endl;
    }
  }

  //****** 14chaps 4.4 ******//
  {
    std::cerr << "****** 14chaps4.4 ******" << std::endl;
    {
      //#1. 李代数、向量到反对称矩阵、反对称矩阵到向量
      std::cerr << "#1. 李代数、向量到反对称矩阵、反对称矩阵到向量"
                << std::endl;
      Eigen::AngleAxisd aa_1(M_PI / 4, Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d R(aa_1);
      Eigen::Quaterniond quat(aa_1);
      Sophus::SO3d SO3_R(quat); // SO3_R(R), SO3_R(aa_1)都可以
      std::cerr << "SO3_R = \n" << SO3_R.matrix() << std::endl;
      auto so3 = SO3_R.log();
      std::cerr << "so3 = SO3_R.log() type is " << typeid(so3).name()
                << ", which is acutally vector3d" << std::endl;
      auto so3Hat = Sophus::SO3d::hat(so3);
      std::cerr << "so3 = " << so3.transpose() << std::endl;
      std::cerr << "so3.hat = \n" << so3Hat << std::endl;
      std::cerr << "so3.hat.vee = " << Sophus::SO3d::vee(so3Hat).transpose()
                << std::endl;
      std::cerr << "exp(so3) = \n"
                << Sophus::SO3d::exp(so3).matrix() << std::endl;

      //#2. SE3相关
      std::cerr << "#2. SE3相关" << std::endl;
      Eigen::Vector3d t(25, 35, 2);
      Sophus::SE3d SE3_1(R, t);
      std::cerr << "SE3_1.matrix() = \n" << SE3_1.matrix() << std::endl;
      auto se3_1 = SE3_1.log();
      std::cerr << "se3_1.type = " << typeid(se3_1).name() << std::endl;
      std::cerr << "se3_1 = " << se3_1.transpose() << std::endl;
      auto SE3_2 = Sophus::SE3d::exp(se3_1);
      std::cerr << "SE3_2.type = " << typeid(SE3_2).name() << std::endl;
      std::cerr << "SE3_2.matrix() = \n" << SE3_2.matrix() << std::endl;

      Eigen::Vector3d t_updating(3.5, 0, 0);
      Eigen::Matrix3d R_updating = Eigen::Matrix3d::Identity();
      Eigen::Matrix<double, 6, 1> updating2 =
          Eigen::Matrix<double, 6, 1>::Zero();
      updating2(0, 0) = 3.5;
      auto SE3_updating_1 = Sophus::SE3d(R_updating, t_updating);
      auto SE3_updating_2 = Sophus::SE3d::exp(updating2);
      std::cerr << "SE3_updating_1 = \n"
                << SE3_updating_1.matrix() << std::endl;
      std::cerr << "SE3_updating_2 = \n"
                << SE3_updating_2.matrix() << std::endl;
      auto SE3_updated_1 = SE3_updating_1 * SE3_1;
      auto SE3_updated_2 = SE3_updating_2 * SE3_1;
      std::cerr << "SE3_updated_1.matrix() = \n"
                << SE3_updated_1.matrix() << std::endl;
      std::cerr << "SE3_updated_2.matrix() = \n"
                << SE3_updated_2.matrix() << std::endl;
      
      // 这里回顾AddOdomPose
      // motion_loc = R.inverse() * motion_odom * R;
      Eigen::Matrix3d odom_R = Eigen::Matrix3d::Identity();
      Eigen::Vector3d odom_t_1(1, 0, 0);
      Eigen::Vector3d odom_t_2(1.5, 0, 0);
      Sophus::SE3d SE3_odom_1(odom_R, odom_t_1);
      Sophus::SE3d SE3_odom_2(odom_R, odom_t_2);
      Sophus::SE3d motion = SE3_odom_1.inverse() * SE3_odom_2;

      Eigen::AngleAxisd loc_aa(M_PI / 4, Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d loc_R = loc_aa.matrix();
      Eigen::Vector3d loc_t(13, 13, 2);
      Sophus::SE3d tf(loc_R, Eigen::Vector3d::Zero());
      Sophus::SE3d loc_1(loc_R, loc_t);
      auto motion_loc = tf * motion * tf.inverse();
      auto loc_2 = motion_loc * loc_1;
      std::cerr << "loc_1.mat = \n" << loc_1.matrix() << std::endl;
      std::cerr << "loc_2.mat = \n" << loc_2.matrix() << std::endl;

      // 计算pose的差
      Eigen::Vector3d pose1_t(1, 0, 0);
      Eigen::Vector3d pose2_t(1, 0, 1);
      Eigen::AngleAxisd pose1_aa(M_PI / 4, Eigen::Vector3d::UnitZ());
      auto pose1_R = pose1_aa.matrix();
      Eigen::Matrix3d pose2_R = Eigen::Matrix3d::Identity();
      Sophus::SE3d pose_1(pose1_R, pose1_t);
      Sophus::SE3d pose_2(pose2_R, pose2_t);
      auto motion_pose = pose_1.inverse() * pose_2;
      std::cerr << "motion_pose.mat = \n" << motion_pose.matrix() << std::endl;
      std::cerr << "motion_pose.log = " << motion_pose.log() << std::endl;
      std::cerr << "motion_pose.log.norm = " << motion_pose.log().norm()
                << std::endl;
      // 这样直接计算整个se3的norm感觉还是不太合理，都不是一个量纲的
    }

    //#3. 扰动量更新
    {
      std::cerr << "#3. 扰动量更新" << std::endl;
      Eigen::AngleAxisd aa_origin(M_PI / 2, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond quat(aa_origin);
      Sophus::SO3d SO3_origin(quat);
      Eigen::Vector3d update_so3(0, 0, M_PI / 4);
      Sophus::SO3d update_SO3 = Sophus::SO3d::exp(update_so3);
      std::cerr << "type update_SO3 = " << typeid(update_SO3).name()
                << std::endl;
      Sophus::SO3d SO3_R_after_update = update_SO3 * SO3_origin;
      std::cerr << "R before update = \n" << SO3_origin.matrix() << std::endl;
      std::cerr << "R after update = \n"
                << SO3_R_after_update.matrix() << std::endl;
      std::cerr << "R.log() before update = " << SO3_origin.log().transpose()
                << std::endl;
      std::cerr << "R.log() after update = "
                << SO3_R_after_update.log().transpose() << std::endl;
    }

    //#4. 结合eskf的SO3预测和更新
    {
      std::cerr << "#4. 结合eskf的SO3预测和更新" << std::endl;
      //##4.1. 预测部分： e.g. IMU读数是a、g，时间为dt
      Eigen::Vector3d a(0.1, 0.1, 0.1);
      Eigen::Vector3d g(0.01, 0.01, 0.01);
      double dt = 0.01;
      Sophus::SO3d R_old =
          Sophus::SO3d::exp(Eigen::Vector3d(0.01, 0.015, M_PI / 4));
      Sophus::SO3d predict = Sophus::SO3d::exp(g * dt);
      // Sophus::SO3d R_new_2 = predict * R_old;
      Sophus::SO3d R_new_1 = R_old * predict;
      // std::cerr << "R_new_2.log = " << R_new_2.log().transpose() << std::endl;
      std::cerr << "R_new_1.log = " << R_new_1.log().transpose() << std::endl;
      // 注意，左扰动和右扰动不同，但数量级很小时结果相近。之后可以一直用右扰动。
      //##4.2. 更新部分：e.g. GPS给出的SO3为 SO3_gps
      Sophus::SO3d R_gps =
          Sophus::SO3d::exp(Eigen::Vector3d(0.02, 0.02, M_PI / 4 + 0.02));
      Eigen::Vector3d ErrorVec = (R_new_1.inverse() * R_gps).log();
      std::cerr << "ErrorVec = " << ErrorVec << std::endl;
      double k = 0.9;
      Eigen::Vector3d UpdateVec = ErrorVec * k;

      // 所以，实际存在状态量中的选装量为R，旋转矩阵/SO3。计算error时用SO3的乘法后转换为so3，计算更新量时用K*so3，更新时再转回SO3用乘法更新
    }

    //#5. 结合eskf的Quaternion预测和更新，暂时不看了
    { std::cerr << "#4. 结合eskf的Quaternion预测和更新" << std::endl; }
  }

  //****** 14chaps chap3 ******//
  {
    std::cerr << "****** 14chaps chap3 ******" << std::endl;
    { 
      std::cerr << std::endl; 
    }
  }
}
}  // namespace LibrariesUeben