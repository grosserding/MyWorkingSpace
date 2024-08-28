#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K) {
  return cv::Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                     (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2,
                          std::vector<cv::KeyPoint>& keypoints_1,
                          std::vector<cv::KeyPoint>& keypoints_2,
                          std::vector<cv::DMatch>& matches) {}

// 2d-2d estimate camera movements
int main(int argc, char&& argv) { return 0; }