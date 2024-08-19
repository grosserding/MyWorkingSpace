#include <ceres/ceres.h>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <opencv2/core/core.hpp>

// fitting curve: y = exp(ax^2 + bx + c)

// 代价函数的计算模型
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
  // calculate residual
  template <typename T>  // what is T?
  bool operator()(const T* const abc,
                  T* residual) const {  // 函数调用运算符 的重载
    // abc 为模型参数，有3维
    residual[0] =
        T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
    return true;
  }
  const double _x, _y;
};

int main(int argc, char** argv) {
  double a = 1.0, b = 2.0, c = 1.0;  // abc真值
  int N = 100;
  double w_sigma = 1.0;                // 噪声的方差
  cv::RNG rng;                         // OpenCV随机数产生器
  double abc[3] = {0, 0, 0};           // abc估计值，初始值为000
  std::vector<double> x_data, y_data;  // 数据
  std::cerr << "generating data: " << std::endl;
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.emplace_back(x);
    y_data.emplace_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
    std::cerr << x_data[i] << " " << y_data[i] << std::endl;
  }

  // 构建最小二乘问题
  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(
        // 自动求导<自定义误差类型, 输出维度, 输入维度>
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1,
                                        3>(  // 添加误差项
            new CURVE_FITTING_COST(x_data[i],
                                   y_data[i])  // 给自定义误差类型添加数据
            ),
        nullptr,  // 核函数，这里不使用，为空
        abc       // 待估计参数
    );
  }

  // 配置求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;  // 增量方程求解
  options.minimizer_progress_to_stdout = true;   // 把进度输出到cout

  ceres::Solver::Summary summary;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::cerr << "time cost: "
            << std::chrono::duration_cast<std::chrono::duration<double>>(t2 -
                                                                         t1)
                   .count()
            << std::endl;

  // 输出结果
  std::cerr << summary.BriefReport() << std::endl;
  std::cerr << "estimated a, b, c = " << abc[0] << " " << abc[1] << " "
            << abc[2] << std::endl;

  //   整理过程：
  //   1.
  //   定义CostFunction模型，并在类中定义带模板参数的()运算符，这样该类成了一个拟函数(Functor)。
  //   比如该类A的对象a，可以调用a<double>()方法。使对象具有函数行为。
  //   2. 调用AddResidualBlock将误差项加入到目标函数。求导方式有
  //   ceres的autodiff自动求导，numericdiff数值求导，解析求导。这里用了自动求导。ceres的自动求导也是数值导数。
  //   3. 自动求导要指定误差项维度、优化变量维度。这里误差：1，维度：3。
  //   4. 调用solve函数求解。可以再option配置迭代次数、步长等。
}