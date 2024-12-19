#define FMT_HEADER_ONLY
#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <random>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>

#include "fmt/format.h"
int ZweiteSuchen(std::vector<double> &nums, const double &obj) {
  int left = 0, right = nums.size() - 1;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    if (nums[mid] == obj) {
      return mid;
    } else if (nums[mid] < obj) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }
  nums.insert(nums.begin() + left, obj);
  std::cout << "nums size +1, now is " << nums.size() << std::endl;
  return -1;
}

int main(int argc, char **argv) {
  // random value generation
  // std::random_device rd;
  // std::mt19937 gen(rd());
  // // normal distribution
  // std::normal_distribution<double> normal_dist(0.0, 1.0);
  // std::uniform_real_distribution<double> uniform_dist(0, 100);
  // std::cout << "i\tnorm_dist\tuni_dist\n";
  // for(int i = 0; i < 100; i++) {
  //     std::cout << i << "\t" << normal_dist(gen) << "\t" << uniform_dist(gen)
  //     << std::endl;
  // }

  // zweite suchen
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> ud(0, 10);
  std::vector<double> nums;
  for (int i = 0; i < 10; i++) {
    nums.emplace_back(ud(gen));
  }
  std::sort(nums.begin(), nums.end());
  std::cout << "nums = ";
  for (auto tmp : nums) {
    std::cout << tmp << ",";
  }
  std::cout << std::endl;
  double insert = ud(gen);
  int search_idx = 5;
  double search_obj = nums[search_idx];
  std::cout << "idx should be " << search_idx << ", func returns "
            << ZweiteSuchen(nums, search_obj) << std::endl;
  search_obj = nums[search_idx] - 0.001;
  std::cout << "idx should be -1, func returns "
            << ZweiteSuchen(nums, search_obj) << std::endl;

  // newton fuer solving
  // kern equation:
  // xj = xi - f(xi)/f'(xi);
  {
    double a = 1.23;
    double sq_a = a * a;
    double xi = 0.01;
    double dx = 1.0;
    double xj;
    int counter = 0;
    while (std::fabs(dx) > 0.000001) {
      xj = xi - (xi * xi - sq_a) / (2 * xi);
      dx = xj - xi;
      xi = xj;
      counter++;
    }
    std::cout << "a = " << a << ", solved a = " << xi
              << ", counter = " << counter << std::endl;

    double a_dritte = sq_a * a;
    xi = 0.01;
    dx = 1.0;
    counter = 0;
    while (std::fabs(dx) > 0.000001) {
      xj = xi - (xi * xi * xi - a_dritte) / (3 * xi * xi);
      dx = xj - xi;
      xi = xj;
      counter++;
    }
    std::cout << "a = " << a << ", solved a = " << xi
              << ", counter = " << counter << std::endl;
  }
  // 1grad/ 2grad(newton) fuer optimization
  // F(x) = ||f(x + dx)||2, F(x) tailor
  // y = exp(ax2 + bx + c)
  {
    double a = 2.232, b = 5.344, c = 0.32;
    double ea = 1.0, eb = 1.0, ec = 1.0;
    std::vector<double> xs, ys;
    xs.reserve(20);
    ys.reserve(20);
    for (int i = 0; i < 20; i++) {
      double j = ((double)i) / 100.0;
      xs.emplace_back(j);
      ys.emplace_back(exp(a * j * j + b * j + c));
      std::cout << "yi = " << exp(a * j * j + b * j + c) << "\n";
    }
    int counter = 0;
    while (counter < 1000) {
      int j = counter % 20;
      double xi = xs[j];
      double yi = ys[j];
      double dea = 2 * (yi - exp(ea * xi * xi + eb * xi + ec)) *
                   (-exp(ea * xi * xi + eb * xi + ec)) * xi * xi;
      double deb = 2 * (yi - exp(ea * xi * xi + eb * xi + ec)) *
                   (-exp(ea * xi * xi + eb * xi + ec)) * xi;
      double dec = 2 * (yi - exp(ea * xi * xi + eb * xi + ec)) *
                   (-exp(ea * xi * xi + eb * xi + ec));
      std::cout << "j = " << j << ", xi = " << xi << ", yi = " << yi
                << ", dea = " << dea << "\n";
      if (fabs(dea) < 0.001 && fabs(deb) < 0.001 && fabs(dec) < 0.001) {
        break;
      } else {
        ea += dea;
        eb += deb;
        ec += dec;
      }
      counter++;
    }
    std::cout << "a = " << a << ", b = " << b << ", c = " << c
              << "\nea = " << ea << ", eb = " << eb << ", ec = " << ec
              << "\ncounter = " << counter << std::endl;
    // 不对
  }

  // gaussnewton
  // F(x) = ||f(x+dx)||2
  // f(x+dx) taylor
  {
    std::cout << "***** gauss-newton test 1" << std::endl;
    double a = 2.232, b = 5.344, c = 0.32;
    double ea = 1.0, eb = 1.0, ec = 1.0;
    std::vector<double> xs, ys;
    xs.reserve(20);
    ys.reserve(20);
    for (int i = 0; i < 20; i++) {
      double j = ((double)i) / 100.0;
      xs.emplace_back(j);
      ys.emplace_back(exp(a * j * j + b * j + c));
    }
    int counter = 0;
    int lastcost = 0;
    while (counter < 1000) {
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      Eigen::Vector3d g = Eigen::Vector3d::Zero();
      double cost = 0;
      for (int i = 0; i < xs.size(); i++) {
        double xi = xs[i];
        double yi = ys[i];
        Eigen::Vector3d J;
        double error = yi - exp(ea * xi * xi + eb * xi + ec);
        J << -exp(ea * xi * xi + eb * xi + ec) * xi * xi,
            -exp(ea * xi * xi + eb * xi + ec) * xi,
            -exp(ea * xi * xi + eb * xi + ec);
        H += J * J.transpose();
        g += -J * error;
        cost += error * error;
      }
      Eigen::Vector3d dx = H.ldlt().solve(g);
      if (std::isnan(dx(0))) {
        std::cout << "result is nan" << std::endl;
        break;
      }
      //   if (counter && cost > lastcost) {
      //     std::cout << "cost no longer decrease, iteration ends" <<
      //     std::endl; break;
      //   }
      if (dx.norm() <= 1e-5) {
        std::cout << "dx is under 1e-5, iteration ends." << std::endl;
        break;
      }
      ea += dx(0);
      eb += dx(1);
      ec += dx(2);
      std::cout << "counter = " << counter << ", dx = " << dx.transpose()
                << ", cost = " << cost << std::endl;
      counter++;
      lastcost = cost;
    }
    std::cout << "a = " << a << ", b = " << b << ", c = " << c
              << "\nea = " << ea << ", eb = " << eb << ", ec = " << ec
              << "\ncounter = " << counter << std::endl;
  }

  // gaussnewton
  // F(x) = ||f(x+dx)||2
  // f(x+dx) taylor
  {
    std::cout << "***** gauss-newton test 3d" << std::endl;
    // ax + by + cz + d = 0;
    double a = 1.0, b = 2.0, c = 3.0, d = 4.0;
    std::vector<double> xs, ys, zs;
    xs.reserve(100);
    ys.reserve(100);
    zs.reserve(100);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> ud(-100, 100);
    std::normal_distribution<double> nd(0, 1);
    std::cout << "generating square:\nx, y, z" << std::endl;
    for (int i = 0; i < 100; i++) {
      double x = ud(gen);
      double y = ud(gen);
      double z = -(a * x + b * y + d + nd(gen)) / c;
      xs.emplace_back(x);
      ys.emplace_back(y);
      zs.emplace_back(z);
      std::cout << x << ", " << y << ", " << z << std::endl;
    }
    double ea = 10, eb = 10, ec = 10, ed = 10;
    for (int counter = 0; counter < 1000; counter++) {
      Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
      Eigen::Vector4d g = Eigen::Vector4d::Zero();
      double error;
      for (int i = 0; i < xs.size(); i++) {
        double x = xs[i], y = ys[i], z = zs[i];
        error = ea * x + eb * y + ec * z + ed;
        Eigen::Vector4d J;
        J << x, y, z, 1;
        H += J * J.transpose();
        g += -J * error;
      }
      Eigen::Vector4d dx = H.ldlt().solve(g);
      if (std::isnan(dx(0))) {
        std::cout << "solving got nan." << std::endl;
        break;
      }
      if (counter > 100 && dx.norm() < 1e-6) {
        std::cout << "dx under 1e-6, iteration ends." << std::endl;
        break;
      }
      ea += dx(0);
      eb += dx(1);
      ec += dx(2);
      ed += dx(3);
      std::cout << "counter = " << counter << ", dx = " << dx.transpose()
                << ", error = " << error << std::endl;
    }
    std::cout << "a = " << a << ", b = " << b << ", c = " << c << ", d = " << d
              << "\nea = " << ea << ", eb = " << eb << ", ec = " << ec
              << ", ed = " << ed << std::endl;
    // wrong, get all zero
  }
  // gaussnewton
  // F(x) = ||f(x+dx)||2
  // f(x+dx) taylor
  {
    std::cout << "***** gauss-newton test 3d 2nd" << std::endl;
    // ax + by + cz + d = 0;
    double a = 1.0, b = 2.0, c = 3.0;
    std::vector<double> xs, ys, zs;
    xs.reserve(100);
    ys.reserve(100);
    zs.reserve(100);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> ud(-100, 100);
    std::normal_distribution<double> nd(0, 1);
    std::cout << "generating square:\nx, y, z" << std::endl;
    for (int i = 0; i < 100; i++) {
      double x = ud(gen);
      double y = ud(gen);
      double z = a * x + b * y + c + nd(gen);
      xs.emplace_back(x);
      ys.emplace_back(y);
      zs.emplace_back(z);
      std::cout << x << ", " << y << ", " << z << std::endl;
    }
    double ea = 10, eb = 10, ec = 10;
    for (int counter = 0; counter < 1000; counter++) {
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      Eigen::Vector3d g = Eigen::Vector3d::Zero();
      double error;
      for (int i = 0; i < xs.size(); i++) {
        double x = xs[i], y = ys[i], z = zs[i];
        error = z - (ea * x + eb * y + ec);
        Eigen::Vector3d J;
        J << -x, -y, -1;
        H += J * J.transpose();
        g += -J * error;
      }
      Eigen::Vector3d dx = H.ldlt().solve(g);
      if (std::isnan(dx(0))) {
        std::cout << "solving got nan." << std::endl;
        break;
      }
      if (dx.norm() < 1e-6) {
        std::cout << "dx under 1e-6, iteration ends." << std::endl;
        break;
      }
      ea += dx(0);
      eb += dx(1);
      ec += dx(2);
      std::cout << "counter = " << counter << ", dx = " << dx.transpose()
                << ", error = " << error << std::endl;
    }
    std::cout << "a = " << a << ", b = " << b << ", c = " << c
              << "\nea = " << ea << ", eb = " << eb << ", ec = " << ec
              << std::endl;
  }
  // gaussnewton self make again
  {
    // ax + by + cz + d = 0 -> z = Ax + By + C
    // f(param) = Ax + By + C
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> ud(-100, 100);
    std::normal_distribution<double> nd(0, 1);
    std::vector<double> x_vec, y_vec, z_vec;
    double A = 1, B = 2, C = 3;
    for (int i = 0; i < 100; i++) {
      double x = ud(gen);
      double y = ud(gen);
      double z = A * x + B * y + C + nd(gen);
      x_vec.emplace_back(x);
      y_vec.emplace_back(y);
      z_vec.emplace_back(z);
    }
    double Ae = 0.1, Be = 0.1, Ce = 0.1;
    // f = Z - (Ae x + Be y + Ce)
    for (int counter = 0; counter < 1000; counter++) {
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      Eigen::Vector3d g = Eigen::Vector3d::Zero();
      for (int i = 0; i < x_vec.size(); i++) {
        double x = x_vec[i], y = y_vec[i], z = z_vec[i];
        Eigen::Vector3d J(-x, -y, -1);
        H += J * J.transpose();
        double error = z - (Ae * x + Be * y + Ce);
        g += -J * error;
      }
      Eigen::Vector3d dx = H.ldlt().solve(g);
      if (std::isnan(dx(0))) {
        std::cout << "dx isnan, break." << std::endl;
        break;
      }
      if (dx.norm() <= 1e-10) {
        std::cout << "dx.norm <= 1e-10, break." << std::endl;
        break;
      }
      Ae += dx(0);
      Be += dx(1);
      Ce += dx(2);
      std::cout << "counter = " << counter << ", dx = " << dx.transpose()
                << ", Ae/Be/Ce = " << Ae << "/" << Be << "/" << Ce << std::endl;
    }
  }
  {
    // 1-grad least square for fitting lanes
    std::vector<double> xs, ys;
    double a = 1.0, b = 2.0;
    double ae = 0.1, be = 0.1;
    int points_num = 100;
    xs.reserve(points_num);
    ys.reserve(points_num);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> ud(-100, 100);
    std::normal_distribution<double> nd(0, 1);
    for (int i = 0; i < points_num; i++) {
      double x = ud(gen);
      double y = a * x + b + nd(gen);
      xs.emplace_back(x);
      ys.emplace_back(y);
    }
    // f(x) = y - (Ax + B) F(x) = ||f(x)||2
    // J 为 F(x) 对 A、B求偏导，得到J(0),J(1)
    // Grad = 0 时，表示可以直接解出A,B
    // J(0) = -2*(y - Ax - B)*x = 0, J(1) = -2*(y - Ax - B) = 0
    // -x^2 * A - x * B + xy = 0
    // -x * A - B + y = 0
    // 转化成矩阵方程求解 Hx = g，这样就不再存在迭代
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    Eigen::Vector2d g = Eigen::Vector2d::Zero();
    for (int i = 0; i < xs.size(); i++) {
      double x = xs[i];
      double y = ys[i];
      Eigen::Matrix2d H_plus;
      Eigen::Vector2d g_plus;
      H_plus << x * x, x, x, 1;
      g_plus << x * y, y;
      H += H_plus;
      g += g_plus;
    }
    Eigen::Vector2d res = H.ldlt().solve(g);
    std::cout << "res = " << res.transpose() << std::endl;
  }
  {
    // NOT using gaussnewton least sqaure fitting plane
    // 思路是，让雅可比的各项等于0！然后直接求解析解，而不再迭代
    // z = ax + by + c
    // f(x, y) = z - ax - by - c
    // F = (z - ax - by - c)^2
    // J0 = 2 * (z - ax - by - c) (-x) = 0
    // J1 = 2 * (z - ax - by - c) (-y) = 0
    // J2 = 2 * (z - ax - by - c) (-1) = 0
    // HX = B
    std::vector<double> xs, ys, zs;
    double a = 1, b = 2, c = 3;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> ud(-100, 100);
    std::normal_distribution<double> nd(0, 1);
    for (int i = 0; i < 100; i++) {
      double x = ud(gen);
      double y = ud(gen);
      double z = a * x + b * y + c + nd(gen);
      xs.emplace_back(x);
      ys.emplace_back(y);
      zs.emplace_back(z);
    }
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d G = Eigen::Vector3d::Zero();
    for (int i = 0; i < xs.size(); i++) {
      double x = xs[i];
      double y = ys[i];
      double z = zs[i];
      Eigen::Matrix3d H_plus;
      Eigen::Vector3d G_plus;
      H_plus << x * x, x * y, x, x * y, y * y, y, x, y, 1;
      G_plus << z * x, z * y, z;
      H += H_plus;
      G += G_plus;
    }
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    Eigen::Vector3d params1 = H.lu().solve(G);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    Eigen::Vector3d params2 = H.llt().solve(G);
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    Eigen::Vector3d params3 = H.ldlt().solve(G);
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    Eigen::Vector3d params4 = H.colPivHouseholderQr().solve(G);
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    Eigen::Vector3d params5 =
        H.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(G);
    std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
    std::cout << "params1 = " << params1.transpose() << ", takes "
              << std::chrono::duration_cast<std::chrono::duration<double>>(t2 -
                                                                           t1)
                     .count()
              << std::endl;
    std::cout << "params2 = " << params2.transpose() << ", takes "
              << std::chrono::duration_cast<std::chrono::duration<double>>(t3 -
                                                                           t2)
                     .count()
              << std::endl;
    std::cout << "params3 = " << params3.transpose() << ", takes "
              << std::chrono::duration_cast<std::chrono::duration<double>>(t4 -
                                                                           t3)
                     .count()
              << std::endl;
    std::cout << "params4 = " << params4.transpose() << ", takes "
              << std::chrono::duration_cast<std::chrono::duration<double>>(t5 -
                                                                           t4)
                     .count()
              << std::endl;
    std::cout << "params5 = " << params5.transpose() << ", takes "
              << std::chrono::duration_cast<std::chrono::duration<double>>(t6 -
                                                                           t5)
                     .count()
              << std::endl;
  }
  {
    // test eigen
    Eigen::Vector3d a(0.1, 0.2, 0.3);
    Eigen::Vector3d b(0.5, 1, 1.5);
    Eigen::Matrix3d A = Sophus::SO3d::exp(a).matrix();
    Eigen::Matrix3d B = Sophus::SO3d::exp(b).matrix();
    auto AB = A * B;
    auto BA = B * A;
    std::cout << "A = " << A << std::endl;
    std::cout << "B = " << B << std::endl;
    std::cout << "AB = " << AB << std::endl;
    std::cout << "BA = " << BA << std::endl;
  }

  double test[3] = {1.0, 2.5, 3.6};
  double *test_ptr = &(test[0]);
  Eigen::Vector3d testout = Eigen::Vector3d(test_ptr);
  std::cout << "test = " << testout.transpose() << std::endl;
  return 1;
}