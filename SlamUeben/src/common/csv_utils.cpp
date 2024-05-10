#include "common/csv_utils.hpp"

CsvIOHelper::CsvIOHelper() {}
CsvIOHelper::~CsvIOHelper() {
  for (auto &file : out_streams_) {
    std::unique_lock<std::timed_mutex> per_file_lock(
        *property_mutex_[file.first], std::defer_lock);
    if (per_file_lock.try_lock_for(std::chrono::milliseconds(1))) {
      file.second.flush();
      file.second.close();
    } else {
      std::cerr << "[CsvIOHelper] fail to get lock, close fail" << std::endl;
    }
  }
}