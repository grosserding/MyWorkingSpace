#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <boost/format.hpp>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
// boost format for formating strings
#include <boost/format.hpp>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// Sophus
#include <sophus/common.hpp>
#include <sophus/se3.hpp>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main(int argc, char **argv) {
  std::vector<cv::Mat> colorImgs, depthImgs;
  std::vector<Sophus::SE3d> poses;

  std::ifstream fin("../data/slam14/kap5/pose.txt");
  if (!fin) {
    std::cerr << "please run this program under path with pose.txt within."
              << std::endl;
    return 1;
  } else {
    std::cerr << "loaded pose.txt" << std::endl;
  }

  for (int i = 0; i < 5; i++) {
    boost::format fmt("../data/slam14/kap5/%s/%d.%s");
    colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
    depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(),
                                   -1));  // 使用-1读取原始图像
    cv::imshow("color_" + std::to_string(i), colorImgs[i]);
    cv::imshow("depth_" + std::to_string(i), depthImgs[i]);
    // cv::waitKey(0);
    double data[7] = {0};
    for (auto &d : data) {
      fin >> d;
    }
    Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
    auto t = Eigen::Vector3d(data[0], data[1], data[2]);
    Sophus::SE3d T(q, t);
    poses.emplace_back(T);
  }

  // join map
  // internal parameter
  double cx = 325.5;
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;
  double depthScale = 1000.0;
  std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
  // std::vector<Eigen::Vector3d>
  // 会有问题，因为Eigen管理内存的方法和c++11标准不一样
  std::cerr << "Generating Pointcloud.." << std::endl;

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  PointCloudT::Ptr pc_ptr(new PointCloudT());
  // here test std::vector<Eigen::Vector3d>: 没测出个所以然
  // std::vector<Eigen::Vector3d> a;
  // a.emplace_back(Eigen::Vector3d(0, 0, 0));
  // a.emplace_back(Eigen::Vector3d(1, 0, 0));
  // a.emplace_back(Eigen::Vector3d(3, 0, 0));
  // std::cerr << "a[0] = " << a[0].transpose() << std::endl;
  // std::cerr << "a[1] = " << a[1].transpose() << std::endl;
  // std::cerr << "a[2] = " << a[2].transpose() << std::endl;

  // read images
  for (int i = 0; i < 5; i++) {
    std::cerr << "converting images: " << i + 1 << std::endl;
    cv::Mat color = colorImgs[i];
    cv::Mat depth = depthImgs[i];
    Sophus::SE3d T = poses[i];
    for (int v = 0; v < color.rows; v++) {
      for (int u = 0; u < color.cols; u++) {
        unsigned int d = depth.ptr<unsigned short>(v)[u];
        if (d == 0) continue;
        Eigen::Vector3d point;
        // 利用相机内参cx, cy, fx, fy获得相机坐标系下的点的坐标，z, x, y
        point[2] = double(d) / depthScale;
        point[0] = (u - cx) * point[2] / fx;
        point[1] = (v - cy) * point[2] / fy;
        // T为相机位置，T*p 得到世界系下的点坐标
        Eigen::Vector3d pointworld = T * point;
        Vector6d p;
        p.head<3>() = pointworld;
        p[5] = color.data[v * color.step + u * color.channels()];      // blue
        p[4] = color.data[v * color.step + u * color.channels() + 1];  // green
        p[3] = color.data[v * color.step + u * color.channels() + 2];  // red
        pointcloud.emplace_back(p);

        // Using PointCloudT
        PointT point_t;
        point_t.x = p[0];
        point_t.y = p[1];
        point_t.z = p[2];
        point_t.r = p[3];
        point_t.g = p[4];
        point_t.b = p[5];
        pc_ptr->points.push_back(point_t);
      }
    }
  }
  std::cerr << "pointcloud has " << pc_ptr->size() << " points." << std::endl;
  pcl::io::savePCDFileBinary("../data/slam14/kap5/generated_map.pcd", *pc_ptr);
  return 0;
}