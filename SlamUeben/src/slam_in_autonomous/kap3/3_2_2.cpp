#include "slam_in_autonomous/kap3/imu_integration.h"

int main(int argc, char** argv) {
  // 给定零偏
  Eigen::Vector3d gravity(0, 0, -9.8);  // 重力方向
  Eigen::Vector3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
  Eigen::Vector3d init_ba(-0.165205, 0.0926887, 0.0058049);

  auto ii_ptr = std::make_shared<IMUIntegration>(
      IMUIntegration(gravity, init_bg, init_ba));

  auto save_result = [](std::ofstream& fout, const double& timestamp,
                        const Sophus::SO3d& R, const Eigen::Vector3d& v,
                        const Eigen::Vector3d& p) {
    auto save_vec3 = [](std::ofstream& fout, const Eigen::Vector3d& vec) {
      fout << vec(0) << "," << vec(1) << "," << vec(2) << ",";
    };
    auto save_quat = [](std::ofstream& fout, const Eigen::Quaterniond& q) {
      fout << q.w() << "," << q.x() << "," << q.y() << "," << q.z();
    };
    fout << std::setprecision(18) << timestamp << "," << std::setprecision(6);
    save_vec3(fout, p);
    save_vec3(fout, v);
    save_quat(fout, R.unit_quaternion());
    fout << std::endl;
  };

  // 这里，在捕获列表里面天剑save_result这个lambda函数对象，就能够在里面调用了
  auto read_data = [save_result](std::ifstream& fin, std::ofstream& fout,
                      std::shared_ptr<IMUIntegration> ii_ptr) {
    if (!fin) {
      std::cout << "no such file.";
      return;
    }
    while (!fin.eof()) {
      std::string line;
      std::getline(fin, line);
      if (line.empty()) {
        continue;
      }
      if (line[0] == '#') {
        continue;
      }
      std::stringstream ss;
      ss << line;
      std::string data_type;
      ss >> data_type;
      if (data_type == "IMU") {
        double time, gx, gy, gz, ax, ay, az;
        ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
        ii_ptr->AddIMU(IMU(time, Eigen::Vector3d(gx, gy, gz),
                           Eigen::Vector3d(ax, ay, az)));
        save_result(fout, time, ii_ptr->GetR(), ii_ptr->GetV(), ii_ptr->GetP());
      }
    }
  };

  std::ofstream fout("../data/sia/kap3/state.csv");
  fout << "stamp,px,py,pz,vx,vy,vz,qw,qx,qy,qz" << std::endl;
  std::ifstream fin("../data/sia/kap3/10.txt");
  read_data(fin, fout, ii_ptr);
  std::cout << "end 3_2_2.\n";
}