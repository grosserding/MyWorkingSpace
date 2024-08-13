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
  std::cerr << "image.cols = " << image.cols << ", image.rows = " << image.rows
            << ", image.channels() = " << image.channels() << std::endl;
  cv::imshow("image", image);
  cv::waitKey(0);
  if (image.type() == CV_8UC1) {
    std::cerr << "CV_8UC1 = " << image.type() << std::endl;
  } else if (image.type() == CV_8UC3) {
    std::cerr << "CV_8UC3 = " << image.type() << std::endl;
    // 彩色图应该是UC3，灰度图是UC1
  }

  std::cerr << "--------- print image ------------" << std::endl;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  for (size_t x = 0; x < image.rows; x++) {
    for (size_t y = 0; y < image.cols; y++) {
      unsigned char* row_ptr = image.ptr<unsigned char>(x);  // row_ptr, x行
      unsigned char* data_ptr = &row_ptr[y];
      for (int c = 0; c < image.channels(); c++) {
        unsigned char data = data_ptr[c];
        // std::cerr << std::to_string(data);
      }
      // std::cerr << " ";
    }
    // std::cerr << ";";
  }
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::cerr << "go through image takes"
            << std::chrono::duration_cast<std::chrono::duration<double>>(t2 -
                                                                         t1)
                   .count()
            << " sec." << std::endl;
  cv::Mat image_shalowclone = image;
  cv::Mat image_deepclone = image.clone();
  image(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("origin", image);
  cv::imshow("deepclone", image_deepclone);
  cv::imshow("shallow", image_shalowclone);
  cv::waitKey(0);
  // cv::destroyAllWindows();          
  return 0;
}