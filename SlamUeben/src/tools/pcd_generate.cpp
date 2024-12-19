#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 定义一个结构来存储二维坐标点
struct PointXY {
    float x;
    float y;
};

// 读取文件并解析其中的 x, y 数据
std::vector<PointXY> readXYFile(const std::string& filename) {
    std::ifstream infile(filename);
    std::string line;
    std::vector<PointXY> points;

    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return points;
    }

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        PointXY point;
        char comma; // 用于跳过逗号
        if (iss >> point.x >> comma >> point.y) {
            points.push_back(point);
        }
    }
    
    infile.close();
    return points;
}

// 将二维坐标点转换为 PCL 点云并保存为 PCD 文件
void createPCDFile(const std::vector<PointXY>& points, const std::string& output_pcd) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 遍历 points，将每个二维点的 z 坐标设置为 0，添加到 PCL 点云中
    for (const auto& point : points) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = 0.0f; // 假设 z 坐标为 0
        cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // 保存为 PCD 文件
    if (pcl::io::savePCDFileASCII(output_pcd, *cloud) == -1) {
        std::cerr << "Error saving PCD file: " << output_pcd << std::endl;
    } else {
        std::cout << "Saved " << cloud->points.size() << " points to " << output_pcd << std::endl;
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_xy_file> <output_pcd_file>" << std::endl;
        return -1;
    }

    // 读取输入的坐标文件
    std::string input_filename = argv[1];
    std::string output_pcd_filename = argv[2];

    // 解析文件，获取二维点集
    std::vector<PointXY> points = readXYFile(input_filename);
    if (points.empty()) {
        std::cerr << "No valid points read from the file." << std::endl;
        return -1;
    }

    // 将数据生成 PCD 点云文件
    createPCDFile(points, output_pcd_filename);

    return 0;
}