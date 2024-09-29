// 7.8.1 使用EPnP求解位姿

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
    // std::cerr << "match.distance = " << match.distance << std::endl;
    if (match.distance <= std::max((2 * min_dist), 30.0)) {
      good_matches.push_back(match);
    }
  }
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cout << "need 2 RGBs and 2 depths.\n";
    return 1;
  }
  cv::Mat mat1 = cv::imread(argv[1]);
  cv::Mat mat2 = cv::imread(argv[2]);
  std::vector<cv::KeyPoint> kp1;
  std::vector<cv::KeyPoint> kp2;
  std::vector<cv::DMatch> matches;
  find_feature_matches(mat1, mat2, kp1, kp2, matches);
  std::cout << "found " << matches.size() << " matches.\n";

  cv::Mat d1 = cv::imread(argv[3]);
  cv::Mat K =
      (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  std::vector<cv::Point3f> pts_3d;
  std::vector<cv::Point2f> pts_2d;
  for (auto match : matches) {
    float d = d1.ptr<unsigned short>(
        int(kp1[match.queryIdx].pt.y))[int(kp1[match.queryIdx].pt.x)];
    if (d == 0)
      continue;
    float dd = d / 1000.0;
    cv::Point2d p1 = pixel2cam(kp1[match.queryIdx].pt,
                               K); // 这里返回的是归一化坐标 p1.x, p1.y, 1
    pts_3d.emplace_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
    pts_2d.emplace_back(kp2[match.trainIdx].pt);
  }
  std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;
  cv::Mat r, t;
  bool success = cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false,
                              cv::SOLVEPNP_EPNP);
  std::cout << "success = " << success << std::endl;
  cv::Mat R;
  cv::Rodrigues(r, R);
  std::cout << "R = " << R << std::endl;
  std::cout << "t = " << t << std::endl;

  return 0;
}