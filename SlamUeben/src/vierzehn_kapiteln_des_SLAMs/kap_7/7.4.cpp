#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
  return cv::Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                     (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
                          std::vector<cv::KeyPoint> &keypoints_1,
                          std::vector<cv::KeyPoint> &keypoints_2,
                          std::vector<cv::DMatch> &good_matches) {
  // 2. find Oriented FAST keypoints
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);
  std::cerr << "found " << keypoints_1.size() << " keypoints_1, "
            << keypoints_2.size() << " keypoints_2." << std::endl;

  // 3. calculate BRIEF descriptor
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Mat descriptors_1, descriptors_2;
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  // 4. show Keypoints on images
  cv::Mat outimg1, outimg2;
  cv::drawKeypoints(img_1, keypoints_1, outimg1, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DEFAULT);
  cv::drawKeypoints(img_2, keypoints_2, outimg2, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DEFAULT);

  // 5. match BRIEF using Hamming distance
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::DMatch> matches;
  matcher->match(descriptors_1, descriptors_2, matches);
  std::cerr << "matches size = " << matches.size() << std::endl;
  // 6. filter useful matched points
  double min_dist = 10000, max_dist = 0;
  min_dist = min_element(matches.begin(), matches.end(),
                         [](const cv::DMatch &m1, const cv::DMatch &m2) {
                           return m1.distance < m2.distance;
                         })
                 ->distance;
  max_dist = max_element(matches.begin(), matches.end(),
                         [](const cv::DMatch &m1, const cv::DMatch &m2) {
                           return m1.distance < m2.distance;
                         })
                 ->distance;
  printf("max dist = %f\n", max_dist);
  printf("min_dist = %f\n", min_dist);
  // when discriptors distance larger than 2*min_dist, match is wrong
  for (auto match : matches) {
    std::cerr << "match.distance = " << match.distance << std::endl;
    if (match.distance <= std::max((2 * min_dist), 30.0)) {
      good_matches.push_back(match);
    }
  }
}

void pose_estimation_2d2d(std::vector<cv::KeyPoint> kpts_1,
                          std::vector<cv::KeyPoint> kpts_2,
                          std::vector<cv::DMatch> matches, cv::Mat &R,
                          cv::Mat &t) {
  // 相机内参，其中cv::Mat_为模板类，可以定义数据类型和尺寸
  cv::Mat K =
      ((cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1));
  // 把匹配点KeyPoint转换为普通的 cv::Point2f
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;
  for (int i = 0; i < (int)matches.size(); i++) {
    points1.push_back(kpts_1[matches[i].queryIdx].pt);
    points2.push_back(kpts_2[matches[i].trainIdx].pt);
  }

  // calculate E, EssentialMatrix 本质矩阵, E = t^R,
  // 八点法求本质矩阵，不包含内参
  cv::Mat eMat;
  cv::Point2d principal_point(325.1, 249.7); // 相机光心（内参标定值）
  double focal_length = 521; // 相机焦距（内参标定值）
  eMat = cv::findEssentialMat(points1, points2, focal_length, principal_point);
  std::cout << "essential matrix is \n" << eMat << std::endl;
  // calculate F, FundamentalMatrix 基础矩阵, F = K^(-T) E K^(-1)
  cv::Mat fMat;
  fMat = cv::findFundamentalMat(points1, points2);
  std::cout << "fundamental matrix is \n" << fMat << std::endl;

  // get pose from essential matrix or fundamental matrix
  cv::recoverPose(eMat, points1, points2, R, t, focal_length, principal_point);
  std::cout << "R = \n" << R << std::endl;
  std::cout << "t = \n" << t << std::endl;
}

// 2d-2d estimate camera movements
int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "need two parameter: pic 1 file path and pic 2 file path.\n";
    return 1;
  }
  cv::Mat img_1 = cv::imread(argv[1]);
  cv::Mat img_2 = cv::imread(argv[2]);
  assert(img_1.data && img_2.data && "cant load imgs");
  std::cout << "images loaded.\n";
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  std::vector<cv::DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  std::cout << matches.size() << " matches found." << std::endl;

  cv::Mat R, t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  return 0;
}