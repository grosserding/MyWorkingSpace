#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <ceres/ceres.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <chrono>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

template<typename T> 
inline T DotProduct(const T x[3], const T y[3]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}
 
template<typename T>
inline void CrossProduct(const T x[3], const T y[3], T result[3]){
  result[0] = x[1] * y[2] - x[2] * y[1];
  result[1] = x[2] * y[0] - x[0] * y[2];
  result[2] = x[0] * y[1] - x[1] * y[0];
}

// Converts from a angle anxis to quaternion : 
template<typename T>
inline void AngleAxisToQuaternion(const T* angle_axis, T* quaternion){
  const T& a0 = angle_axis[0];
  const T& a1 = angle_axis[1];
  const T& a2 = angle_axis[2];
  const T theta_squared = a0 * a0 + a1 * a1 + a2 * a2;
  
  
  if(theta_squared > T(std::numeric_limits<double>::epsilon()) ){
    const T theta = sqrt(theta_squared);
    const T half_theta = theta * T(0.5);
    const T k = sin(half_theta)/theta;
    quaternion[0] = cos(half_theta);
    quaternion[1] = a0 * k;
    quaternion[2] = a1 * k;
    quaternion[3] = a2 * k;
  }
  else{ // in case if theta_squared is zero
    const T k(0.5);
    quaternion[0] = T(1.0);
    quaternion[1] = a0 * k;
    quaternion[2] = a1 * k;
    quaternion[3] = a2 * k;
  }
}
 
 
template<typename T>
inline void QuaternionToAngleAxis(const T* quaternion, T* angle_axis){
  const T& q1 = quaternion[1];
  const T& q2 = quaternion[2];
  const T& q3 = quaternion[3];
  const T sin_squared_theta = q1 * q1 + q2 * q2 + q3 * q3;
  
  // For quaternions representing non-zero rotation, the conversion
  // is numercially stable
  if(sin_squared_theta > T(std::numeric_limits<double>::epsilon()) ){
    const T sin_theta = sqrt(sin_squared_theta);
    const T& cos_theta = quaternion[0];
    
    // If cos_theta is negative, theta is greater than pi/2, which
    // means that angle for the angle_axis vector which is 2 * theta
    // would be greater than pi...
    
    const T two_theta = T(2.0) * ((cos_theta < 0.0)
				  ? atan2(-sin_theta, -cos_theta)
				  : atan2(sin_theta, cos_theta));
    const T k = two_theta / sin_theta;
    
    angle_axis[0] = q1 * k;
    angle_axis[1] = q2 * k;
    angle_axis[2] = q3 * k;
  }
  else{
    // For zero rotation, sqrt() will produce NaN in derivative since
    // the argument is zero. By approximating with a Taylor series, 
    // and truncating at one term, the value and first derivatives will be 
    // computed correctly when Jets are used..
    const T k(2.0);
    angle_axis[0] = q1 * k;
    angle_axis[1] = q2 * k;
    angle_axis[2] = q3 * k;
  }
  
}
 
 
template<typename T>
inline void AngleAxisRotatePoint(const T angle_axis[3], const T pt[3], T result[3]) {
  const T theta2 = DotProduct(angle_axis, angle_axis);
  if (theta2 > T(std::numeric_limits<double>::epsilon())) {
    // Away from zero, use the rodriguez formula
    //
    //   result = pt costheta +
    //            (w x pt) * sintheta +
    //            w (w . pt) (1 - costheta)
    //
    // We want to be careful to only evaluate the square root if the
    // norm of the angle_axis vector is greater than zero. Otherwise
    // we get a division by zero.
    //
    const T theta = sqrt(theta2);
    const T costheta = cos(theta);
    const T sintheta = sin(theta);
    const T theta_inverse = 1.0 / theta;
 
    const T w[3] = { angle_axis[0] * theta_inverse,
                     angle_axis[1] * theta_inverse,
                     angle_axis[2] * theta_inverse };
 
    // Explicitly inlined evaluation of the cross product for
    // performance reasons.
    /*const T w_cross_pt[3] = { w[1] * pt[2] - w[2] * pt[1],
                              w[2] * pt[0] - w[0] * pt[2],
                              w[0] * pt[1] - w[1] * pt[0] };*/
    T w_cross_pt[3];
    CrossProduct(w, pt, w_cross_pt);                          
 
 
    const T tmp = DotProduct(w, pt) * (T(1.0) - costheta);
    //    (w[0] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * (T(1.0) - costheta);
 
    result[0] = pt[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
    result[1] = pt[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
    result[2] = pt[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;
  } else {
    // Near zero, the first order Taylor approximation of the rotation
    // matrix R corresponding to a vector w and angle w is
    //
    //   R = I + hat(w) * sin(theta)
    //
    // But sintheta ~ theta and theta * w = angle_axis, which gives us
    //
    //  R = I + hat(w)
    //
    // and actually performing multiplication with the point pt, gives us
    // R * pt = pt + w x pt.
    //
    // Switching to the Taylor expansion near zero provides meaningful
    // derivatives when evaluated using Jets.
    //
    // Explicitly inlined evaluation of the cross product for
    // performance reasons.
    /*const T w_cross_pt[3] = { angle_axis[1] * pt[2] - angle_axis[2] * pt[1],
                              angle_axis[2] * pt[0] - angle_axis[0] * pt[2],
                              angle_axis[0] * pt[1] - angle_axis[1] * pt[0] };*/
    T w_cross_pt[3];
    CrossProduct(angle_axis, pt, w_cross_pt); 
 
    result[0] = pt[0] + w_cross_pt[0];
    result[1] = pt[1] + w_cross_pt[1];
    result[2] = pt[2] + w_cross_pt[2];
  }
}

struct ICPCeres {
  ICPCeres(Point3f uvw, Point3f xyz) : _uvw(uvw), _xyz(xyz) {}
  // 残差的计算
  template <typename T>
  bool operator()(const T* const camera,  // 模型参数，有4维
                  T* residual) const      // 残差
  {
    T p[3];
    T point[3];
    point[0] = T(_xyz.x);
    point[1] = T(_xyz.y);
    point[2] = T(_xyz.z);
    AngleAxisRotatePoint(camera, point, p);  //计算RP
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];  //相机坐标2
    residual[0] = T(_uvw.x) - p[0];
    residual[1] = T(_uvw.y) - p[1];
    residual[2] = T(_uvw.z) - p[2];
    return true;
  }
  static ceres::CostFunction* Create(const Point3f uvw, const Point3f xyz) {
    return (new ceres::AutoDiffCostFunction<ICPCeres, 3, 6>(
        new ICPCeres(uvw, xyz)));
  }
  const Point3f _uvw;
  const Point3f _xyz;
};
void find_feature_matches(const Mat& img_1, const Mat& img_2,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d& p, const Mat& K);

void pose_estimation_3d3d(const vector<Point3f>& pts1,
                          const vector<Point3f>& pts2, Mat& R, Mat& t);
int main(int argc, char** argv) {
  vector<Point3f> pts1, pts2;
  double camera[6] = {0, 1, 2, 0, 0, 0};
  pts1.push_back(Point3f(-107.339447,368.235229,7.203767));
  pts1.push_back(Point3f(-13.938293,-3.508911,18.397972));
  pts1.push_back(Point3f(36.682861,444.794983,-2.003768));
  pts2.push_back(Point3f(-106.496429,369.124573,6.662053));
  pts2.push_back(Point3f(0.446471,0.734270,-1.074847));
  pts2.push_back(Point3f(35.070869,450.609863,7.601254));

  cout << "3d-3d pairs: " << pts1.size() << endl;
  Mat R, t;
  pose_estimation_3d3d(pts1, pts2, R, t);
  cout << "ICP via SVD results: " << endl;
  cout << "R = " << R << endl;
  cout << "t = " << t << endl;
  cout << "R_inv = " << R.t() << endl;
  cout << "t_inv = " << -R.t() * t << endl;

  cout << "calling bundle adjustment" << endl;
  // verify p1 = R*p2 + t
  for (int i = 0; i < 5; i++) {
    cout << "p1 = " << pts1[i] << endl;
    cout << "p2 = " << pts2[i] << endl;
    cout << "(R*p2+t) = "
         << R * (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t
         << endl;
    cout << endl;
  }
  ceres::Problem problem;
  for (int i = 0; i < pts2.size(); ++i) {
    ceres::CostFunction* cost_function = ICPCeres::Create(pts2[i], pts1[i]);
    problem.AddResidualBlock(cost_function, NULL /* squared loss */, camera);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  Mat R_vec =
      (Mat_<double>(3, 1) << camera[0], camera[1], camera[2]);  //数组转cv向量
  Mat R_cvest;
  Rodrigues(R_vec, R_cvest);  //罗德里格斯公式，旋转向量转旋转矩阵
  cout << "R_cvest=" << R_cvest << endl;
  Eigen::Matrix<double, 3, 3> R_est;
  cv2eigen(R_cvest, R_est);  // cv矩阵转eigen矩阵
  cout << "R_est=" << R_est << endl;
  Eigen::Vector3d t_est(camera[3], camera[4], camera[5]);
  cout << "t_est=" << t_est << endl;
  Eigen::Isometry3d T(R_est);  //构造变换矩阵与输出
  T.pretranslate(t_est);
  cout << T.matrix() << endl;

  return 0;
}
void find_feature_matches(const Mat& img_1, const Mat& img_2,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches) {
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
  vector<DMatch> match;
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

Point2d pixel2cam(const Point2d& p, const Mat& K) {
  return Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                 (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void pose_estimation_3d3d(const vector<Point3f>& pts1,
                          const vector<Point3f>& pts2, Mat& R, Mat& t) {
  Point3f p1, p2;  // center of mass
  int N = pts1.size();
  for (int i = 0; i < N; i++) {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 = Point3f(Vec3f(p1) / N);
  p2 = Point3f(Vec3f(p2) / N);
  vector<Point3f> q1(N), q2(N);  // remove the center
  for (int i = 0; i < N; i++) {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // compute q1*q2^T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (int i = 0; i < N; i++) {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) *
         Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  cout << "W=" << W << endl;

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  cout << "U=" << U << endl;
  cout << "V=" << V << endl;

  Eigen::Matrix3d R_ = U * (V.transpose());
  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) -
                       R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

  // convert to cv::Mat
  R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2), R_(1, 0), R_(1, 1),
       R_(1, 2), R_(2, 0), R_(2, 1), R_(2, 2));
  t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

// 代价函数，计算单个点 a_i 和 b_i 的残差
// struct PoseCostFunctor {
//   PoseCostFunctor(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
//       : a_(a), b_(b) {}

//   template <typename T>
//   bool operator()(const T* const quaternion, const T* const translation,
//                   T* residual) const {
//     // 将四元数数据转换为 Eigen::Quaternion
//     Eigen::Quaternion<T> q(quaternion[0], quaternion[1], quaternion[2],
//                            quaternion[3]);
//     q.normalize();

//     // 将 a 点进行旋转和平移
//     Eigen::Matrix<T, 3, 1> t(translation[0], translation[1], translation[2]);
//     Eigen::Matrix<T, 3, 1> a_transformed = q * a_.cast<T>() + t;

//     // 计算残差
//     Eigen::Map<Eigen::Matrix<T, 3, 1>>(residual) = a_transformed -
//     b_.cast<T>(); return true;
//   }

//   const Eigen::Vector3d a_;
//   const Eigen::Vector3d b_;
// };

// int main() {
//   // 示例点集 A 和 B
//   Eigen::Vector3d a1 = Eigen::Vector3d(-107.339447, 368.235229, 7.203767);
//   Eigen::Vector3d a2 = Eigen::Vector3d(-13.938293, -3.508911, 18.397972);
//   Eigen::Vector3d a3 = Eigen::Vector3d(36.682861, 444.794983, -2.003768);
//   Eigen::Vector3d b1 = Eigen::Vector3d(-106.496429, 369.124573, 6.662053);
//   Eigen::Vector3d b2 = Eigen::Vector3d(0.446471, 0.734270, -1.074847);
//   Eigen::Vector3d b3 = Eigen::Vector3d(35.070869, 450.609863, 7.601254);
//   std::vector<Eigen::Vector3d> A = {a1, a2, a3};
//   std::vector<Eigen::Vector3d> B = {b1, b2, b3};

//   // 初始化四元数和平移
//   Eigen::Quaterniond quaternion(1, 0, 0, 0);  // 单位四元数
//   Eigen::Vector3d translation(0, 0, 0);

//   // 构建 Ceres 问题
//   ceres::Problem problem;
//   for (size_t i = 0; i < A.size(); ++i) {
//     ceres::CostFunction* cost_function =
//         new ceres::AutoDiffCostFunction<PoseCostFunctor, 3, 4, 3>(
//             new PoseCostFunctor(A[i], B[i]));
//     problem.AddResidualBlock(cost_function, nullptr,
//     quaternion.coeffs().data(),
//                              translation.data());
//   }

//   // 配置求解器
//   ceres::Solver::Options options;
//   options.linear_solver_type = ceres::DENSE_QR;
//   options.minimizer_progress_to_stdout = true;

//   // 增加单位四元数约束，确保优化过程中四元数始终为单位四元数
//   problem.SetParameterization(quaternion.coeffs().data(),
//                               new ceres::QuaternionParameterization());

//   // 求解问题
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);

//   // 输出优化结果
//   quaternion.normalize();
//   Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
//   std::cout << "Final Rotation: \n" << rotation << std::endl;
//   std::cout << "Final Translation: \n" << translation.transpose() <<
//   std::endl;

//   Sophus::SE3d T(rotation, translation);

//   std::cerr << "a1_t = " << (T * a1).transpose() << std::endl;
//   std::cerr << "b1 = " << (b1).transpose() << std::endl;
//   std::cerr << "a2_t = " << (T * a2).transpose() << std::endl;
//   std::cerr << "b2 = " << (b2).transpose() << std::endl;
//   std::cerr << "a3_t = " << (T * a3).transpose() << std::endl;
//   std::cerr << "b3 = " << (b3).transpose() << std::endl;

//   return 0;
// }

// 用四元数优化向量
// class EuclideanTransformResidual {
//  public:
//   EuclideanTransformResidual(const Eigen::Vector3d& a, const Eigen::Vector3d&
//   b)
//       : a_(a), b_(b) {}

//   template <typename T>
//   bool operator()(const T* const q, const T* const t, T* residual) const {
//     Eigen::Quaternion<T> quat(q);
//     Eigen::Matrix<T, 3, 1> translation(t);
//     Eigen::Matrix<T, 3, 1> a_t = a_.cast<T>();
//     Eigen::Matrix<T, 3, 1> b_t = b_.cast<T>();

//     // 将四元数转换为旋转矩阵
//     Eigen::Matrix<T, 3, 3> R = quat.toRotationMatrix();
//     // 计算残差
//     residual[0] = (R * a_t + translation - b_t).squaredNorm();
//     return true;
//   }

//  private:
//   const Eigen::Vector3d a_;
//   const Eigen::Vector3d b_;
// };

// int main() {
//   //   // 输入向量组 a 和 b
//   Eigen::Vector3d a1 = Eigen::Vector3d(-107.339447, 368.235229, 7.203767);
//   Eigen::Vector3d a2 = Eigen::Vector3d(-13.938293, -3.508911, 18.397972);
//   Eigen::Vector3d a3 = Eigen::Vector3d(36.682861, 444.794983, -2.003768);
//   Eigen::Vector3d b1 = Eigen::Vector3d(-106.496429, 369.124573, 6.662053);
//   Eigen::Vector3d b2 = Eigen::Vector3d(0.446471, 0.734270, -1.074847);
//   Eigen::Vector3d b3 = Eigen::Vector3d(35.070869, 450.609863, 7.601254);
//   std::vector<Eigen::Vector3d> a = {a1, a2, a3};
//   std::vector<Eigen::Vector3d> b = {b1, b2, b3};

//   // 初始化四元数（单位四元数）和位移
//   Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
//   Eigen::Vector3d t = Eigen::Vector3d::Zero();

//   // 优化问题的设置
//   ceres::Problem problem;
//   for (size_t i = 0; i < a.size(); ++i) {
//     ceres::CostFunction* cost_function =
//         new ceres::AutoDiffCostFunction<EuclideanTransformResidual, 1, 4, 3>(
//             new EuclideanTransformResidual(a[i], b[i]));
//     problem.AddResidualBlock(cost_function, nullptr, q.coeffs().data(),
//                              t.data());
//   }

//   // 设置优化器
//   ceres::Solver::Options options;
//   options.dynamic_sparsity = true;
//   options.max_num_iterations = 10000;
//   options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
//   options.minimizer_type = ceres::TRUST_REGION;
//   options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//   options.trust_region_strategy_type = ceres::DOGLEG;
//   options.minimizer_progress_to_stdout = true;
//   options.dogleg_type = ceres::SUBSPACE_DOGLEG;
//   ceres::Solver::Summary summary;

//   // 运行优化
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << "\n";

//   // 输出结果
//   q.normalize();
//   Eigen::Matrix3d R = q.toRotationMatrix();
//   std::cout << "Estimated rotation matrix:\n" << R << "\n";
//   std::cout << "Estimated translation vector:\n" << t << "\n";

//   Sophus::SE3d T(R, t);
//   std::cerr << "a1_t = " << (T * a1).transpose() << std::endl;
//   std::cerr << "b1 = " << (b1).transpose() << std::endl;
//   std::cerr << "a2_t = " << (T * a2).transpose() << std::endl;
//   std::cerr << "b2 = " << (b2).transpose() << std::endl;
//   std::cerr << "a3_t = " << (T * a3).transpose() << std::endl;
//   std::cerr << "b3 = " << (b3).transpose() << std::endl;

//   return 0;
// }

// 用旋转矩阵优化向量
// // 定义残差类
// class EuclideanTransformResidual {
//  public:
//   EuclideanTransformResidual(const Eigen::Vector3d& a, const Eigen::Vector3d&
//   b)
//       : a_(a), b_(b) {}

//   template <typename T>
//   bool operator()(const T* const r, const T* const t, T* residual) const {
//     Eigen::Map<const Eigen::Matrix<T, 3, 3>> R(r);
//     Eigen::Map<const Eigen::Matrix<T, 3, 1>> Translate(t);
//     Eigen::Matrix<T, 3, 1> a_t = a_.cast<T>();
//     Eigen::Matrix<T, 3, 1> b_t = b_.cast<T>();

//     residual[0] = (R * a_t + Translate - b_t).squaredNorm();
//     return true;
//   }

//   //   bool operator()(const double* const r, const double* const t, double*
//   //   residual) const {
//   //     Eigen::Map<const Eigen::Matrix<double, 3, 3>> R(r);
//   //     Eigen::Map<const Eigen::Matrix<double, 3, 1>> T(t);
//   //     Eigen::Matrix<double, 3, 1> a_t = a_.cast<double>();
//   //     Eigen::Matrix<double, 3, 1> b_t = b_.cast<double>();

//   //     residual[0] = (R * a_t + T - b_t).squaredNorm();
//   //     return true;
//   //   }

//  private:
//   const Eigen::Vector3d a_;
//   const Eigen::Vector3d b_;
// };

// int main() {
//   // 输入向量组 a 和 b
//   Eigen::Vector3d a1, a2, a3, b1, b2, b3;
//   a1 << -50.6212, -448.304, 20.4017;
//   a2 << 93.4012, -371.744, 11.1942;
//   a3 << 144.022, 76.5598, -9.20754;
//   b1 << -34.6244, -449.876, -8.6761;
//   b2 << 106.943, -368.39, -7.7369;
//   b3 << 141.567, 81.4853, 0.939201;
//   std::vector<Eigen::Vector3d> a = {a1, a2, a3};
//   std::vector<Eigen::Vector3d> b = {b1, b2, b3};

//   // 初始化旋转矩阵（旋转角度的四元数）和位移
//   Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
//   Eigen::Vector3d t = Eigen::Vector3d::Zero();

//   // 优化问题的设置
//   ceres::Problem problem;
//   for (size_t i = 0; i < a.size(); ++i) {
//     ceres::CostFunction* cost_function =
//         new ceres::AutoDiffCostFunction<EuclideanTransformResidual, 1, 9, 3>(
//             new EuclideanTransformResidual(a[i], b[i]));
//     problem.AddResidualBlock(cost_function, nullptr, R.data(), t.data());
//   }

//   // 设置优化器
//   ceres::Solver::Options options;
//   options.linear_solver_type = ceres::DENSE_QR;
//   options.minimizer_progress_to_stdout = true;
//   ceres::Solver::Summary summary;

//   // 运行优化
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << "\n";

//   // 输出结果
//   std::cout << "Estimated rotation matrix:\n" << R << "\n";
//   std::cout << "Estimated translation vector:\n" << t << "\n";

//   Sophus::SE3d T(R, t);

//   std::cerr << "a1_t = " << (T * a1).transpose() << std::endl;
//   std::cerr << "b1 = " << (b1).transpose() << std::endl;
//   std::cerr << "a2_t = " << (T * a2).transpose() << std::endl;
//   std::cerr << "b2 = " << (b2).transpose() << std::endl;
//   std::cerr << "a3_t = " << (T * a3).transpose() << std::endl;
//   std::cerr << "b3 = " << (b3).transpose() << std::endl;

//   return 0;
// }