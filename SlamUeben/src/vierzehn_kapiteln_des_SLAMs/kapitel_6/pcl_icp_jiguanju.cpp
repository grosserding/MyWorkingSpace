#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <pcl/console/time.h>  // 利用控制台计算时间
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  // icp算法
#include <pcl/visualization/pcl_visualizer.h>
// Sophus
#include <filesystem>
#include <sophus/common.hpp>
#include <sophus/se3.hpp>

// #include <boost/thread/thread.hpp>
#include <iostream>

using namespace std;

Eigen::Matrix3f orthogonalize(const Eigen::Matrix3f& rotation_matrix) {
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      rotation_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f U = svd.matrixU();
  Eigen::Matrix3f V = svd.matrixV();
  Eigen::Matrix3f orthogonal_matrix = U * V.transpose();
  return orthogonal_matrix;
}

std::vector<std::string> listFilesInDirectory(
    const std::string& directoryPath) {
  std::vector<std::string> files;

  try {
    // 遍历目录中的所有文件和子目录
    for (const auto& entry :
         std::filesystem::recursive_directory_iterator(directoryPath)) {
      if (entry.is_regular_file()) {             // 确保是文件
        files.push_back(entry.path().string());  // 添加文件路径到列表中
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    std::cout << "Filesystem error: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    std::cout << "General error: " << e.what() << std::endl;
  }

  return files;
}

int main(int argc, char** argv) {
  // -------------------获取自制点云文件----------------------
  std::string path =
      "/home/westwell/qpilot_dev_ws/QP_tasks/QP-17109/self_made_pcds";
  std::vector<std::string> file_lists = listFilesInDirectory(path);
  std::cout << "totally " << file_lists.size() << " files.\n";
  sort(file_lists.begin(), file_lists.end());

  // std::vector<std::string> file_lists = listFilesInDirectory(path);
  // for (auto file_tmp : file_lists) {
  //   std::cout << file_tmp << std::endl;
  // }
  // std::cout << std::endl;
  pcl::console::TicToc time;
  // --------------------加载华测点云-----------------------
  pcl::PointCloud<pcl::PointXYZI>::Ptr huace_part(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr huace_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  // for (auto pcd_file : file_lists) {
  //   pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *huace_part);
  //   *huace_cloud += *huace_part;
  //   cout << "从华测点云中读取 " << huace_part->size() << " 个点" << endl;
  //   cout << "现在合并的点云中有 " << huace_cloud->size() << " 个点" << endl;
  // }
  // pcl::io::savePCDFileASCII(
  //     "/home/westwell/qpilot_dev_ws/QP_tasks/QP-17109/huace_pcd/merged.pcd",
  //     *huace_cloud);

  cout << "loading "
          "/home/westwell/qpilot_dev_ws/QP_tasks/QP-17109/huace_pcd/merged.pcd"
       << endl;
  pcl::io::loadPCDFile<pcl::PointXYZI>(
      "/home/westwell/qpilot_dev_ws/QP_tasks/QP-17109/huace_pcd/merged.pcd",
      *huace_cloud);
  cout << "merged huace pc has " << huace_cloud->size() << " points" << endl;

  //--------------------初始化ICP对象--------------------
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  // icp.setInputTarget(huace_cloud);      // 目标点云
  // icp.setCorrespondenceRandomness(20);
  icp.setTransformationEpsilon(1e-10);  // 为终止条件设置最小转换差异
  icp.setMaxCorrespondenceDistance(
      10);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
  icp.setEuclideanFitnessEpsilon(
      0.0001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
  icp.setMaximumIterations(1000);  // 最大迭代次数
  icp.setUseReciprocalCorrespondences(true);  //设置为true,则使用相互对应关系
  // icp.setRANSACOutlierRejectionThreshold(0.1);
  // icp.setRANSACIterations(5);
  // icp.setRANSACOutlierRejectionThreshold(1);

  Eigen::Matrix4f transform;
  transform <<     0.588956 ,   0.808077,  -0.0119553   ,   1121.1,
  -0.807667 ,   0.589048 ,  0.0263928  ,  -711.609,
  0.0283697,-0.00588826  ,   0.99958  ,   7.09358,
          0   ,        0     ,      0   ,        1;
  Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
  // tmp(0, 3) = -1.6;
  // tmp(1, 3) = -0.6;
  transform = tmp * transform;
  double cropbox_delta = 250.0;
  int counter = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_part(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (auto file_tmp : file_lists) {
    if ((counter++) % 2 != 0) continue;
    std::cout << "************** loop head **************" << std::endl;
    size_t pos = file_tmp.find_last_of("/");
    std::string pure_filename = file_tmp.substr(pos + 1);
    std::cout << "now processing: " << pure_filename << std::endl;
    time.tic();
    pcl::PointCloud<pcl::PointXYZI>::Ptr self_made(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(file_tmp, *self_made);
    std::cout << "read " << self_made->size() << " points" << std::endl;
    std::cout << "transform before orthogonalize = \n"
              << transform << std::endl;
    transform.block<3, 3>(0, 0) = orthogonalize(transform.block<3, 3>(0, 0));
    Sophus::SE3f se3_transform(transform);
    std::cout << "transform after orthogonalize = \n" << transform << std::endl;
    pcl::transformPointCloud(*self_made, *self_made, transform);
    icp.setInputSource(self_made);  // 源点云
    std::cout << "pre-process source and setInputSource used "
              << time.toc() / 1000.0 << "s" << std::endl;
    // pcl::io::savePCDFileBinary(
    //     "/home/westwell/qpilot_dev_ws/QP_tasks/QP-17109/"
    //     "tmp" +
    //         pure_filename,
    //     *self_made);

    time.tic();
    auto translation = se3_transform.translation();
    pcl::CropBox<pcl::PointXYZI> crop_box;
    pcl::PointCloud<pcl::PointXYZI>::Ptr huace_cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZI>);
    crop_box.setMin(Eigen::Vector4f(translation(0) - cropbox_delta,
                                    translation(1) - cropbox_delta,
                                    translation(2) - 100.0, 1));
    crop_box.setMax(Eigen::Vector4f(translation(0) + cropbox_delta,
                                    translation(1) + cropbox_delta,
                                    translation(2) + 100.0, 1));
    crop_box.setInputCloud(huace_cloud);
    crop_box.filter(*huace_cloud_filtered);
    // pcl::io::savePCDFileBinary(
    //     "/home/westwell/qpilot_dev_ws/QP_tasks/QP-17109/"
    //     "target_crop_tmp.pcd",
    //     *huace_cloud_filtered);
    std::cout << "before dynamic_part size = "
              << huace_cloud_filtered->points.size() << std::endl;
    *huace_cloud_filtered = *huace_cloud_filtered + *dynamic_part;
    std::cout << "before dynamic_part size = "
              << huace_cloud_filtered->points.size() << std::endl;

    icp.setInputTarget(huace_cloud_filtered);  // 目标点云
    std::cout << "pre-process target and setInputTarget used "
              << time.toc() / 1000.0 << "s" << std::endl;
    time.tic();

    //----------------------icp核心代码--------------------
    // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr icp_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    icp.align(*icp_cloud);
    std::cout << "Applied " << 1000 << " ICP iterations in "
              << ((double)time.toc()) / 1000.0 << "s" << std::endl;
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore()
              << std::endl;
    Eigen::Matrix4f transform_icp = icp.getFinalTransformation();
    // Sophus::SE3f se3_icp(transform_icp);
    std::cout << "se3_icp at " << pure_filename << " is: \n"
              << transform_icp << std::endl;
    // se3_transform = se3_icp * se3_transform;
    std::cout << "transform before = \n" << transform << std::endl;
    transform = transform_icp * transform;
    // std::cout << "transform after = \n" << transform << std::endl;
    Eigen::Quaternionf quat(transform.block<3, 3>(0, 0));
    quat.normalize();
    transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
    std::cout << "FINAL TRANSFORM OF STAMP " << pure_filename << " = \n"
              << transform << std::endl;

    pcl::transformPointCloud(*self_made, *self_made, transform_icp);
    pcl::io::savePCDFileBinary(
        "/home/westwell/qpilot_dev_ws/QP_tasks/QP-17109/"
        "self_made_transformed/trans_" +
            pure_filename,
        *self_made);
    *dynamic_part = *self_made;
  }

  return (0);
}