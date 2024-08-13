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

int main(int argc, char** argv) {
  std::vector<cv::Mat> colorImgs, depthImgs;
  std::vector<Eigen::Isometry3d> poses;

  std::ifstream fin("../data/slam14/kap5/pose.txt");
  if(!fin) {
    std::cerr << "please run this program under path with pose.txt within."
              << std::endl;
    return 1;
  }

  return 0;
}