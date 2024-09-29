// 7.8.2 这里用新书里面的代码对比各种PNP的求解

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/common.hpp>
#include <sophus/se3.hpp>
#include <chrono>
using namespace cv;

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
// 输入两张图，输出他们的keypoints，以及他们的matches，
// 用的都是cv中自带的ORB方法，包括FeatureDetector、DescriptorMatcher等
void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2,
                          std::vector<cv::KeyPoint> &kpts_1,
                          std::vector<cv::KeyPoint> &kpts_2,
                          std::vector<cv::DMatch> &matches);

// 2d-2d对极约束求解姿态
void pose_estimation_2d2d(std::vector<cv::KeyPoint> kpts_1,
                          std::vector<cv::KeyPoint> kpts_2,
                          std::vector<cv::DMatch> matches, cv::Mat &R,
                          cv::Mat &t);

// 像素坐标转相机归一化坐标，注意，返回的点为归一化坐标：x_norm, y_norm, z = 1
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

// 手写GN-BA，输入2d点和对应的3d点，输出pose，K是什么？
void handmadeBAGN(const VecVector3d &pts_3d, const VecVector2d &pts_2d,
                  const cv::Mat &K, Sophus::SE3d &pose);

// g2o的BA，输入2d点和对应的3d点，输出pose，K是什么？
void g2oBA(const VecVector3d &pts_3d, const VecVector2d &pts_2d, const cv::Mat &K,
           Sophus::SE3d &pose);

int main(int argc, char **argv) {
  // ***** 读取图像
  if (argc != 5) {
    std::cout << "param nums wrong";
    return 1;
  }
  cv::Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
  assert(img_1.data && img_2.data && "cant load images!");

  // ***** 获取两张图中的匹配点
  std::vector<cv::KeyPoint> kpts1, kpts2;
  std::vector<cv::DMatch> matches;
  find_feature_matches(img_1, img_2, kpts1, kpts2, matches);
  std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl;

  // ***** 利用深度信息建立3d点
  cv::Mat d1 = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
  // 这里是相机内参
  cv::Mat K =
      (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  std::vector<cv::Point3d> pts_3d;
  std::vector<cv::Point2d> pts_2d;
  double depth_scale = 5000.0;
  for (auto m : matches) {
    // 这里深度信息的获得非常复杂:
    // d1.ptr<short>(int y)[int x];
    ushort d = d1.ptr<unsigned short>(
        int(kpts1[m.queryIdx].pt.y))[int(kpts1[m.queryIdx].pt.x)];
    // 利用深度scale计算实际深度z值
    double dd = d/depth_scale;
    // 利用像素值和内参获取相机坐标系下归一化坐标的pt2d，里面的坐标为归一化坐标x_normalized,
    // y_normalized
    cv::Point2d p1 = pixel2cam(kpts1[m.queryIdx].pt, K);
    // 获取在第一个图像坐标下的3d点
    pts_3d.push_back(
        cv::Point3d(p1.x * dd, p1.y * dd,
                    dd));  // x = x_normalized * z, y = y_normalized * z;
    // 获取第一个图像坐标下的像素坐标
    pts_2d.push_back(kpts1[m.trainIdx].pt);
  }
  std::cout << pts_3d.size() << "3d-2d pairs" << std::endl;
  // 注意因为是做2d-3d，所以只用了第一张深度图

  // ***** 调用Opencv的PnP求解
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  cv::Mat r, t;
  // cv::InputArray objectPoints, cv::InputArray imagePoints, cv::InputArray cameraMatrix, 
  // cv::InputArray distCoeffs, cv::OutputArray rvec, cv::OutputArray tvec, 
  // bool useExtrinsicGuess = false, int flags = 0
  // 因此参数按顺序分别为：3d相机坐标系下坐标，2d像素坐标，内参，一个什么参数，旋转量，平移量，是否用外参猜测
  cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false);
  cv::Mat R;
  cv::Rodrigues(r, R); // r为旋转向量形式，现在转为旋转矩阵R
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve pnp in opencv cost time: " << time_used.count()
            << " seconds." << std::endl;

  std::cout << "R = \n" << R << std::endl;
  std::cout << "t = \n" << t << std::endl;

  // ***** 手写Gauss-newton的BA求解
  // 用之前求解得到的pts_3d和pts_2d来做，获得Eigen定义的点
  VecVector3d pts_3d_eigen;
  VecVector2d pts_2d_eigen;
  for(size_t i = 0; i < pts_3d.size(); i++) {
    pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
    pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
  }
  std::cout << "**** BA by gauss newton *****\n";
  Sophus::SE3d pose_gn;
  t1 = std::chrono::steady_clock::now();
  handmadeBAGN(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
  t2 = std::chrono::steady_clock::now();
  time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve pnp by gauss newton cost time: " << time_used.count()
            << " seconds." << std::endl;

  
  // ***** g2o的BA求解
  std::cout << "**** BA by g2o *****\n";
  Sophus::SE3d pose_g2o;
  t1 = std::chrono::steady_clock::now();
  g2oBA(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
  t2 = std::chrono::steady_clock::now();
  time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << std::endl;

  return 0;
}

// 函数定义如下

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB"
  // );
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  std::vector<DMatch> match;
  // BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离,
  //即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}