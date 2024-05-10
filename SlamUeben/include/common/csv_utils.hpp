#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <string>
#include <utility>
#include <vector>

#include "common/transform.h"
#include "common/singleton.hpp"

class CsvIOHelper {
  DECLARE_SINGLETON(CsvIOHelper)
 private:
  std::map<std::string, std::ofstream> out_streams_;
  std::map<std::string, std::shared_ptr<std::timed_mutex>> property_mutex_;
  std::map<std::string, std::pair<std::string, std::vector<std::string>>>
      file_message_;

 public:
  ~CsvIOHelper();

  bool RegisterFileWithHeader(const std::string& dir, const std::string& name,
                              const std::vector<std::string>& header) {
    auto time =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream strs_time;
    strs_time << std::put_time(std::localtime(&time), "%Y-%m-%d_%H%M%S");
    std::string abs_dir = dir;
    if ('/' != abs_dir.back()) abs_dir.push_back('/');
    std::string file_path = abs_dir + name + "_" + strs_time.str() + ".csv";

    auto iter = file_message_.find(name);
    if (iter != file_message_.end()) {
      auto dir_name = iter->second;
    } else {
      file_message_[name] = std::make_pair(dir, header);
    }

    property_mutex_[name] = std::make_shared<std::timed_mutex>();
    std::unique_lock<std::timed_mutex> per_file_lock(*property_mutex_[name],
                                                     std::defer_lock);
    if (per_file_lock.try_lock_for(std::chrono::milliseconds(1))) {
      out_streams_[name] = std::ofstream();
      out_streams_[name].open(file_path, std::ios::out | std::ios_base::trunc);
      for (size_t i = 0; i < header.size(); i++)
        out_streams_[name] << header.at(i) +
                                  ((i == (header.size() - 1)) ? "\n" : ",");
      return true;
    } else {
      std::cerr << "[CsvIOHelper] fail to get lock, init fail" << std::endl;
      return false;
    }
  }

  std::string format(const std::string& data) { return data; }

  template <typename T>
  std::string format(const T& data) {
    return std::to_string(data);
  }
  template <typename S, int R, int C>
  std::string format(const Eigen::Matrix<S, R, C>& data) {
    std::string tmp;
    for (int i = 0; i < data.rows(); i++) {
      for (int j = 0; j < data.cols(); j++) {
        tmp += std::to_string(data(i, j)) + ",";
      }
    }
    tmp.erase(tmp.size() - 1);
    return tmp;
  }
  template <typename S>
  std::string format(const Sophus::SE2<S>& data) {
    return std::to_string(data.translation().x()) + "," +
           std::to_string(data.translation().y()) + "," +
           std::to_string(data.so2().log());
  }
  template <typename S>
  std::string format(const Sophus::SE3<S>& data) {
    Eigen::Matrix<S, 3, 1> rpy =
        transform::ToRollPitchYaw(data.so3().unit_quaternion());
    return std::to_string(data.translation().x()) + "," +
           std::to_string(data.translation().y()) + "," +
           std::to_string(data.translation().z()) + "," +
           std::to_string(rpy.x()) + "," + std::to_string(rpy.y()) + "," +
           std::to_string(rpy.z());
  }

  template <typename T>
  std::string& generate_line(std::string& os, const T& t) {
    os += format(t) + "\n";
    return os;
  }

  template <typename T, typename... Args>
  std::string& generate_line(std::string& os, const T& t, const Args&... rest) {
    os += format(t) + ",";
    return generate_line(os, rest...);
  }

  template <typename... Args>
  void write_line(const std::string& name, const Args&... content) {
    if ((out_streams_.end() != out_streams_.find(name))) {
      std::unique_lock<std::timed_mutex> per_file_lock(*property_mutex_[name],
                                                       std::defer_lock);
      if (per_file_lock.try_lock_for(std::chrono::milliseconds(1))) {
        if (out_streams_[name].is_open()) {
          std::string tmp;
          std::string out = generate_line(tmp, content...);
          out_streams_[name] << out;
          out_streams_[name].flush();
          out_streams_[name].seekp(0, std::ios::end);
          int file_size = (int)out_streams_[name].tellp();
          int file_size_mb = file_size / (1024 * 1024);
          if (file_size_mb > 10) {
            out_streams_[name].flush();
            out_streams_[name].close();
            auto iter = file_message_.find(name);
            if (iter != file_message_.end()) {
              auto dir_name = iter->second;
              bool reset_file =
                  RegisterFileWithHeader(dir_name.first, name, dir_name.second);
            }
          }
        }
      } else {
        std::cerr << "[CsvIOHelper] fail to get lock, write fail" << std::endl;
      }
    }
  }

  void save() {
    for (auto& file : out_streams_) {
      file.second.flush();
      file.second.close();
    }
  }
};
