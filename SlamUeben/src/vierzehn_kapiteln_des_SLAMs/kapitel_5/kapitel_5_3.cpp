#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
  cv::Mat image;
  image = cv::imread(argv[1]);
  if (image.data == nullptr) {
    std::cerr << "image not valid." << std::endl;
    return 0;
  } else {
    std::cerr << "image" << argv[1] << " loaded." << std::endl;
  }
  return 0;
}