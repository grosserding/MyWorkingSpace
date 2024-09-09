#include <iostream>
#include <vector>
// #include <cstdlib> // 生成随机数的库，包含rand() srand()
// #include <ctime> // 包含time()函数
#include <algorithm>  // sort()
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>
#include <typeinfo>

//##1. zweite suchen
// standard loesung, 给一个值，如果查到返回idx，如果找不到，返回-1并插入。
int Search(std::vector<double> &list, double goal) {
  int right = list.size() - 1, left = 0;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    if (list[mid] == goal) {
      return mid;
    }
    if (list[mid] < goal) {
      left = mid + 1;  // 小的+1大的-1,才能做到最后left > right
    } else {
      right = mid - 1;
    }
  }
  std::cout << "list[left] = " << list[left] << std::endl;
  std::cout << "goal = " << goal << std::endl;
  std::cout << "list[right] = " << list[right] << std::endl;
  list.insert(
      list.begin() + left,
      goal);  // 最后是在left的地方插入，说明list[left]是比goal要大的，也就是最后达到的效果是
              // list[right] < goal < list[left]!本来在右边的right已经跑到左边了
  return -1;
}

int main(int argc, char **argv) {
  std::cout << "# Allgemaine Fragen" << std::endl;
  //# Allgemaine Fragen
  {
    //##1. zweite suchen
    std::cout << "## zweite suchen" << std::endl;
    std::vector<double> list;
    std::random_device rd;
    for (int i = 0; i < 100; i++) {
      auto tmp_rd = rd();
      if (!i) {
        std::cout << typeid(tmp_rd).name() << std::endl;
        // 返回值为"j",表示该值为long或long long
      }
      double tmp = ((double)(tmp_rd / 100000)) / 100;
      std::cout << tmp << ", ";
      list.emplace_back(tmp);
    }
    std::cout << std::endl;
    sort(list.begin(), list.end());
    int n = 15;
    double goal1 = list[n];
    double goal2 = 50.0;
    std::cout << "before Search, list len = " << list.size() << std::endl;
    int idx1 = Search(list, goal1);
    std::cout << "after search 1st, size = " << list.size()
              << ", idx1 = " << idx1 << std::endl;
    int idx2 = Search(list, goal2);
    std::cout << "after search 2nd, size = " << list.size()
              << ", idx2 = " << idx2 << std::endl;
    // // 确定有的查找
    // int left = 0, right = list.size() - 1;
    // while (left <= right) {
    //   int mid = left + (right - left) / 2;
    //   if (list[mid] == goal) {
    //     std::cout << "goal idx = " << mid << std::endl;
    //     break;
    //   }
    //   if (list[mid] < goal) {
    //     left = mid;
    //   } else {
    //     right = mid;
    //   }
    // }
    // // 插入
    // double insert_value = 10000;
    // int goal_idx;
    // left = 0;
    // right = list.size() - 1;
    // bool inserted = false;
    // if (list[left] >= insert_value) {
    //   goal_idx = left;
    //   list.insert(list.begin() + goal_idx, insert_value);
    //   inserted = true;
    // } else if (list[right] <= insert_value) {
    //   goal_idx = list.size();
    //   list.emplace_back(insert_value);
    //   inserted = true;
    // } else {
    //   while (left != right - 1) {
    //     int mid = left + (right - left) / 2;
    //     if (list[mid] == insert_value) {
    //       goal_idx = mid;
    //       list.insert(list.begin() + goal_idx, insert_value);
    //       inserted = true;
    //       break;
    //     } else if (list[mid] < insert_value) {
    //       left = mid;
    //     } else {
    //       right = mid;
    //     }
    //   }
    // }
    // if (!inserted) {
    //   goal_idx = right;
    //   list.insert(list.begin() + goal_idx, insert_value);
    // }
    // std::cout << "inserted position = " << goal_idx << std::endl;
    // for (auto tmp : list) std::cout << tmp << ", ";
    // std::cout << std::endl;

    //** 关于typeid().name()的输出列表：
    // b:bool
    // c:char
    // d:double
    // e:long double
    // f:float
    // i:int32_t
    // j:uint32_t
    // s:int16_t(short)
    // t:uint16_t(unsigned short)
    // x:int64_t
    // y:uint64_t
    // NSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE:string
    // St6vectorI*SaI*EE:类型为 * 的 vector
    // St4pairI*^E：类型分别为 * ^ 的 pair
    // (省流：STL 找子串)
    // A*_-:... 类型的长度为 * 的数组
    // P-:... 的指针
    // PK-:const ... 的指针
  }

  {
    //## alles sortieren(permutation)
    std::cout << "## permutation" << std::endl;
    // normal data
    std::vector<double> list;
    std::random_device rd;
    for (int i = 0; i < 3; i++) {
      list.emplace_back(rd() / (int)1000 / (double)1000);
    }
    std::sort(list.begin(), list.end());
    int counter = 0;
    do {
      std::cout << ++counter << ": ";
      for (auto tmp : list) {
        std::cout << tmp << ", ";
      }
      std::cout << std::endl;
    } while (next_permutation(list.begin(), list.end()));

    std::cout << "### permutation for structed data" << std::endl;
    // special data struct
    struct dataset {
      dataset(double a) { data = a; }
      double data;
    };
    std::vector<dataset> data_list;
    for (int i = 0; i < 4; i++) {
      data_list.emplace_back(dataset(rd() / (int)1000 / (double)1000));
    }
    std::sort(data_list.begin(), data_list.end(),
              [](dataset a, dataset b) -> bool { return (a.data < b.data); });
    counter = 0;
    do {
      std::cout << ++counter << ": ";
      for (auto tmp : data_list) {
        std::cout << tmp.data << ", ";
      }
      std::cout << std::endl;
    } while (next_permutation(
        data_list.begin(), data_list.end(),
        [](dataset a, dataset b) -> bool { return a.data < b.data; }));
  }

  {
    //## newton for solving equations->f(x) = 0
    // sqrt bedeutet x^2 - b = 0
    // kern equation: x_{k+1} = x_k - f(x_k) / f'(x_k)
    std::cout << "## newton for solving equations" << std::endl;
    std::random_device rd;
    double a = (double)rd() / 1e6;
    std::cout << "a = " << a << std::endl;
    double sqrta = sqrt(a);
    std::cout << "sqrt(a) = " << sqrta << ", "
              << "sqrta^2 = " << sqrta * sqrta << std::endl;
    double x0 = 0;
    double x1 = 1;
    int counter = 0;
    do {
      counter++;
      x0 = x1;
      x1 = x0 - (x0 * x0 - a) / (2 * x0);
    } while (abs(x1 * x1 - a) > 0.00001);
    std::cout << ", x1 = " << x1 << ", x1^2 = " << x1 * x1
              << ", counter = " << counter << std::endl;
  }
  double a = 1, b = 2, c = 3;
  std::vector<double> x_list;
  std::vector<double> y_list;
  cv::RNG rng;
  for (double i = -1; i < 1; i += 0.2) {
    x_list.emplace_back(i);
    y_list.emplace_back(exp(a * i * i + b * i + c) + rng.gaussian(1.0));
  }
  {
    //## 1-grad for optimization: min Fx = 1/2 ||fx||^2, Taylor on Fx
    // min Fx -> dx = -Grad
    // dx = -J
    std::cout << "## 1-grad for optimization-> min ||fx||^2" << std::endl;
  }
  {
    //## 2-grad for optimization: min Fx = 1/2 ||fx||^2, Taylor on Fx
    // min Fx -> (Fx)' = 0 -> Fx + JxT dx + dxT Hx dx -> 对dx求导=0
    // H dx = -J
    std::cout << "## 2-grad for optimization-> min ||fx||^2 (Newton)"
              << std::endl;
  }
  {
    //## Handmade Gauss-Newton for optimization: f(x+dx) = f(x) + J(x)dx, Fx = f(x+dx)T
    // f(x+dx)
    // Jx JxT dx = - Jx fx -> H dx = g
    // Eigen::Vector3d test = Eigen::Vector3d::Zero();
    // auto Mattest = test * test.transpose();
    // std::cout << "test.typename = " << typeid(test).name() << std::endl;
    // std::cout << "Mattest.cols = " << Mattest.cols()
    //           << ", Mattest.rows = " << Mattest.rows() << std::endl;
    std::cout << "## Gauss-Newton for optimization-> min ||fx||^2" << std::endl;
    std::cout << "### self wrote" << std::endl;
    const int size = x_list.size();
    Eigen::Vector3d J;
    Eigen::MatrixXd H(size * 3, 3);
    Eigen::MatrixXd g(size * 3, 1);
    double fxi;
    Eigen::Vector3d x_est(1.0, 1.0, 1.0);
    Eigen::Vector3d dx;
    double exp_tmp;
    int maxIters = 100;
    bool satisfied = false;
    int iters = 0;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (; iters < maxIters; iters++) {
      for (int i = 0; i < size; i++) {
        exp_tmp = exp(x_est[0] * x_list[i] * x_list[i] + x_est[1] * x_list[i] +
                      x_est[2]);
        J(2) = -exp_tmp;
        J(1) = J(2) * x_list[i];
        J(0) = J(1) * x_list[i];
        fxi = y_list[i] - exp_tmp;
        H.block<3, 3>(3 * i, 0) = J * J.transpose();
        g.block<3, 1>(3 * i, 0) = -J * fxi;
      }
      dx = H.colPivHouseholderQr().solve(g);
      std::cout << "dx = " << dx << std::endl;
      if (isnan(dx[0])) {
        std::cout << "result is nan!" << std::endl;
        break;
      }
      x_est += dx;
    //   if (dx.norm() < 1e-10) {
    //     satisfied = true;
    //     break;
    //   }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_mine =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "It takes " << time_mine.count() << "sec, iters = " << iters
              << ", x_est = " << x_est.transpose() << std::endl;

    std::cout << "### from Gao" << std::endl;
    // Here, i set Hx = g as so: H(datasize * 3, 3), x(3, 1), g(datasize*3, 1)
    // Yet Gao set Hx = g as: H(3, 3), x(3, 1), g(3, 1), which means he adds up
    // all data into one equation for each iteration.

    Eigen::Vector3d J_gao;
    Eigen::MatrixXd H_gao(3, 3);
    Eigen::MatrixXd g_gao(3, 1);
    x_est << 1.0, 1.0, 1.0;
    iters = 0;
    t1 = std::chrono::steady_clock::now();
    for (; iters < maxIters; iters++) {
      H_gao.setZero();
      g_gao.setZero();
      for (int i = 0; i < size; i++) {
        exp_tmp = exp(x_est[0] * x_list[i] * x_list[i] + x_est[1] * x_list[i] +
                      x_est[2]);
        J_gao(2) = -exp_tmp;
        J_gao(1) = -exp_tmp * x_list[i];
        J_gao(0) = -exp_tmp * x_list[i] * x_list[i];
        H_gao += J_gao * J_gao.transpose();
        fxi = y_list[i] - exp_tmp;
        g_gao += -J_gao * fxi;
      }
      dx = H_gao.colPivHouseholderQr().solve(g_gao);
      if (isnan(dx[0])) {
        std::cout << "result is nan!" << std::endl;
        break;
      }
      x_est += dx;
    //   if (dx.norm() < 1e-10) {
    //     satisfied = true;
    //     break;
    //   }
    }
    t2 = std::chrono::steady_clock::now();
    time_mine =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "It takes " << time_mine.count() << "sec, iters = " << iters
              << ", x_est = " << x_est.transpose() << std::endl;
  }
  {
    //## Handmade LM
  }
  return 0;
}