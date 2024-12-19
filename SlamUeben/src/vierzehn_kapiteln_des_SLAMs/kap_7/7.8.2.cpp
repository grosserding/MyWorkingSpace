// 7.8.2 使用BA优化求解
#include <iostream>

// 观察BA问题的两个点和一个边:
// 1. 第一个顶点是3D点:
// g2o::VertexPointXYZ : public BaseVertex<3, Vector3>
// 1.1. 优化变量是3维，类型为xyz。
// 1.2. oplusImpl()，也就是加法，这里对于平凡的3d点就是加法 _estimate += update;
// 1.3. 看到其他的函数比如setEstimateDataImpl
// getEstimateData等，都使用Eigen::Map
// 这里大概描述以下Eigen::Map的作用：建立与对象之间的关系，复用数组中的内存，建立Eigen数据类型

// 2. 第二个顶点并非是2D点，而是相机位置姿态。2D点在这个问题里面是观测类型。
// g2o::VertexSE3Expmap : public BaseVertex<6, SE3Quat>
// 2.1. 说明是优化变量: 是6维，优化变量类型为SE3Quat即四元数加位移
// 2.2. oplusImpl()，setEstimate(SE3Quat::exp(update) * estimate())
// 这里的加法就是 deltaR * R

// 3. 边:
// class EdgeProjectXYZ2UV : public BaseBinaryEdge<2, Vector2, VertexPointXYZ,
// VertexSE3Expmap> 观测值为2维，Vector2，即像素坐标。

// 看到边的函数实现
// 3.1.
// 计算误差，误差函数这是优化问题的核心。实现的重点就是用顶点类型来构造误差函数Error。
// void EdgeProjectXYZ2UV::computeError(){
//     const VertexSE3Expmap* v1 = static_cast<const
//     VertexSE3Expmap*>(_vertices[1]); 取出位姿顶点 const VertexPointXYZ* v2 =
//     static_cast<const VertexPointXYZ*>(_vertices[0]); 取出3D点顶点 const
//     CameraParameters* cam = static_cast<const
//     CameraParameters*>(parameter(0));
// 先不管了，先把内容抄完
// }

// 明晚 9月6号晚一定要把这段代码抄完
int main(int argc, char **argv) { return 0; }