#include "common/common_include.h"
#include "common/kbhit.h"
#include "common/singleton.hpp"

#define VEL_L 5.0
#define VEL_A 10.0
#define USE_QUAT false
#define COUNT_MAX 60 * 60 * 20

int main(int argc, char** argv) {
  std::string cwd;
  cwd = getcwd(NULL, 0);
  cwd += "/../data/";
  std::cerr << "cwd = " << cwd << std::endl;
  auto& csv_util = CsvIOHelper::GetInstance();
  std::vector<std::string> header{"stamp", "x",     "y",  "z",
                                  "roll",  "pitch", "yaw"};
  std::string csv_name = "kapitel_zwei_circle_movements";
  csv_util.RegisterFileWithHeader(cwd, csv_name, header);
  std::cerr << "Kapitel zwei" << std::endl;
  Sophus::SE3d pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  Eigen::Vector3d v_angle(0, 0, VEL_A / 180 * M_PI);
  Eigen::Vector3d v_linear(VEL_L, 0, 0);
  const double dt = 0.05;
  int ch = 0;
  int count = 0;
  while ('q' != tolower(ch) && count < COUNT_MAX) {
    // calculate pose update on my own
    // Sophus::SO3d rot_last = pose.so3();
    // Eigen::Vector3d translation_last = pose.translation();
    // // 1. 速度坐标转换
    // auto v_map = rot_last * v_linear;
    // // 2. 更新位置
    // auto translation = translation_last + v_map * dt;
    // // 3. 更新旋转
    // auto rot = rot_last * Sophus::SO3d::exp(v_angle * dt);
    // pose = Sophus::SE3d(rot, translation);

    // more tidy one by GX
    auto v_world = pose.so3() * v_linear;
    pose.translation() += v_world * dt;
    pose.so3() = pose.so3() * Sophus::SO3d::exp(v_angle * dt);
    double stamp_now =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count() /
        1e9;  // 注意这里标准的当前的timestamp生成的方式
    std::cerr << "stamp = " << std::to_string(stamp_now)
              << ", translation = " << pose.translation().transpose()
              << ", rpy = " << pose.so3().log().transpose() * 180 / M_PI
              << std::endl;

    csv_util.write_line(csv_name, stamp_now, pose);
    // loop control
    if (kbhit()) {
      ch = getchar();
    }
    count++;
    // std::cerr << "now in loop, dt = "
    //           << std::chrono::duration_cast<std::chrono::nanoseconds>(finish -
    //                                                                    start)
    //                  .count()
    //           << " nanosecs, press 'q' to end loop." << std::endl;
    usleep(dt * 1e6);
  }
  std::cerr << "loop now end." << std::endl;
  return 0;
}