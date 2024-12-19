#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

// feature extraction
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: neeed 2 images" << std::endl;
    return 1;
  }
  // 1. read mat
  cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

  // 2. find Oriented FAST keypoints
  std::vector<cv::KeyPoint> kpts1, kpts2;  // 角点
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  detector->detect(img_1, kpts1);
  detector->detect(img_2, kpts2);
  std::cerr << "found " << kpts1.size() << "kpts1, " << kpts2.size() << "kpts2."
            << std::endl;

  // 3. calculate BRIEF descriptor
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Mat descriptors_1, descriptors_2;
  descriptor->compute(img_1, kpts1, descriptors_1);
  descriptor->compute(img_2, kpts2, descriptors_2);

  // 4. show Keypoints on images
  cv::Mat outimg1, outimg2;
  cv::drawKeypoints(img_1, kpts1, outimg1, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DEFAULT);
  cv::drawKeypoints(img_2, kpts2, outimg2, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DEFAULT);
  cv::imshow("ORB_points on img1", outimg1);
  cv::imshow("ORB_points on img2", outimg2);
  cv::waitKey(0);

  // 5. match BRIEF using Hamming distance
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::DMatch> matches;
  matcher->match(descriptors_1, descriptors_2, matches);

  // 6. filter useful matched points
  double min_dist = 10000, max_dist = 0;
  min_dist = min_element(matches.begin(), matches.end(),
                         [](const cv::DMatch& m1, const cv::DMatch& m2) {
                           return m1.distance < m2.distance;
                         })
                 ->distance;
  max_dist = max_element(matches.begin(), matches.end(),
                         [](const cv::DMatch& m1, const cv::DMatch& m2) {
                           return m1.distance < m2.distance;
                         })
                 ->distance;
  printf("max dist = %f\n", max_dist);
  printf("min_dist = %f\n", min_dist);
  // when discriptors distance larger than 2*min_dist, match is wrong
  std::vector<cv::DMatch> good_matches;
  for (auto match : matches) {
    if (match.distance <= 2 * min_dist) {
      good_matches.push_back(match);
    }
  }

  // 7. draw matches and good matches on images
  cv::Mat img_match;
  cv::drawMatches(img_1, kpts1, img_2, kpts2, matches, img_match);
  cv::imshow("matches", img_match);
  cv::waitKey(0);

  return 0;
}