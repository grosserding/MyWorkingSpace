#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <iostream>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

// SE3 的残差结构
struct SE3Residual {
    SE3Residual(const Eigen::Vector3d& pt_A, const Eigen::Vector3d& pt_B)
        : pt_A_(pt_A), pt_B_(pt_B) {}

    template <typename T>
    bool operator()(const T* const se3, T* residual) const {
        // 将SE3参数转换为旋转和平移
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(se3 + 3);
        Sophus::SO3<T> rotation = Sophus::SO3<T>::exp(Eigen::Map<const Eigen::Matrix<T, 3, 1>>(se3));

        // 将A点通过变换变换到B点
        Eigen::Matrix<T, 3, 1> transformed_pt = rotation * pt_A_.cast<T>() + translation;

        residual[0] = transformed_pt[0] - T(pt_B_[0]);
        residual[1] = transformed_pt[1] - T(pt_B_[1]);
        residual[2] = transformed_pt[2] - T(pt_B_[2]);

        return true;
    }

    Eigen::Vector3d pt_A_;
    Eigen::Vector3d pt_B_;
};

// 使用 Ceres 优化 SE3 变换
Sophus::SE3d EstimateSE3Transform(const std::vector<Eigen::Vector3d>& A, 
                                  const std::vector<Eigen::Vector3d>& B) {
    // SE3 变换参数 [旋转的李代数(3), 平移向量(3)]
    double se3[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 初始估计 [李代数 + 平移]

    ceres::Problem problem;

    for (size_t i = 0; i < A.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<SE3Residual, 3, 6>(new SE3Residual(A[i], B[i]));
        problem.AddResidualBlock(cost_function, nullptr, se3);
    }

    // 设置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // 求解问题
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    // 计算优化得到的 SE3 变换
    Eigen::Matrix<double, 3, 1> translation(se3[3], se3[4], se3[5]);
    Eigen::Matrix<double, 3, 1> so3(se3[0], se3[1], se3[2]);
    Sophus::SE3d transformation(Sophus::SO3d::exp(so3), translation);
    
    return transformation;
}

int main() {
    // 示例数据: 两个点云 (A 和 B)
    std::vector<Eigen::Vector3d> A = {
        {-60.8866,-68.0127, 0.0}, 
        {-4.3407,-47.9868, 0.0}, 
        {-116.3058,-92.0645, 0.0}, 
        {-159.5034,-76.3156, 0.0}, 
        {-148.1796,-19.4835, 0.0}, 
        {-98.2628,-7.6175, 0.0}, 
        {-64.0954,12.7024, 0.0}, 
        {-24.4829,26.4014, 0.0}, 
        {524.3151,-772.3391, 0.0}, 
        {509.699,-741.1702, 0.0}, 
        {496.302,-698.4745, 0.0}, 
        {482.5317,-651.0974, 0.0},         
        {476.4606,-679.6435, 0.0}, 
        {490.0119,-723.1274, 0.0}, 
        {504.5896,-770.0156, 0.0}, 
        {512.4345,-794.902, 0.0}, 
        {538.2957,-784.6528, 0.0}, 
        {1968.926,-131.6174, 0.0}, 
        {2006.3066,-129.5341, 0.0}, 
        {2036.9011,-134.3879, 0.0}, 
        {2043.893,-163.3807, 0.0}, 
        {2055.9907,-198.5274, 0.0}, 
        {2067.0522,-230.3519, 0.0}
    };
    std::vector<Eigen::Vector3d> B = {
        {-1877.13109168736,1313.34736881382, 0.0}, 
        {-1817.12091372721,1313.721549073, 0.0}, 
        {-1937.53382604267,1308.65118263755, 0.0}, 
        {-1972.93856543954,1337.50689292257, 0.0}, 
        {-1943.81993121747,1387.50582209858, 0.0}, 
        {-1892.75110869366,1382.38311183266, 0.0}, 
        {-1853.81113141775,1390.41650028969, 0.0}, 
        {-1811.83248883998,1390.33883892442, 0.0}, 
        {-1554.74223619606,455.303426714148, 0.0}, 
        {-1558.296573475,489.684841666603, 0.0}, 
        {-1556.95671984111,534.358529138146, 0.0}, 
        {-1554.46682828642,583.64502273663, 0.0},         
        {-1569.52647101693,558.694647163851, 0.0}, 
        {-1570.92733760853,513.134658048861, 0.0}, 
        {-1572.54293153109,463.9714273063, 0.0}, 
        {-1573.26689439267,437.967228541151, 0.0}, 
        {-1545.49446017062,439.082872817759, 0.0}, 
        {21.2367770527489,587.739414269803, 0.0}, 
        {57.2668778696098,577.466528669465, 0.0}, 
        {84.6103615271859,562.880699175876, 0.0}, 
        {81.7058738037013,533.145232980605, 0.0}, 
        {81.6204461953603,495.977997873677, 0.0}, 
        {81.6403863064479,462.269117466873, 0.0}
    };

    // 估计 SE3 变换
    Sophus::SE3d transformation = EstimateSE3Transform(A, B);
    Eigen::Vector3d translation = transformation.translation();
    Eigen::Quaterniond quat(transformation.so3().unit_quaternion());
    Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    // 打印结果
    std::cout << "Estimated SE3 Transformation:\n" << transformation.matrix() << std::endl;
    std::cout << "translation :\n" << translation.transpose() << std::endl;
    std::cout << "rpy :\n" << rpy.transpose() << std::endl;

    return 0;
}