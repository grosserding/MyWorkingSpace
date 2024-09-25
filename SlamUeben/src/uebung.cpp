#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <random>
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
    for(int i = 0; i < 100; i++) {
        double x = ud(gen);
        double y = ud(gen);
        double z = A * x + B * y + C + nd(gen);
        x_vec.emplace_back(x);
        y_vec.emplace_back(y);
        z_vec.emplace_back(z);
    }
    double Ae = 0.1, Be = 0.1, Ce = 0.1;
    // f = Z - (Ae x + Be y + Ce)
    for(int counter = 0; counter < 1000; counter++) {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d g = Eigen::Vector3d::Zero();
        for(int i = 0; i < x_vec.size(); i++) {
            double x = x_vec[i], y = y_vec[i], z = z_vec[i];
            Eigen::Vector3d J(-x, -y, -1);
            H += J * J.transpose();
            double error = z - (Ae * x + Be * y + Ce);
            g += - J * error;
        }
        Eigen::Vector3d dx = H.ldlt().solve(g);
        if(std::isnan(dx(0))) {
            std::cout << "dx isnan, break." << std::endl;
            break;
        }
        if(dx.norm() <= 1e-10) {
            std::cout << "dx.norm <= 1e-10, break." << std::endl;
            break;
        }
        Ae += dx(0);
        Be += dx(1);
        Ce += dx(2);
        std::cout << "counter = " << counter << ", dx = " << dx.transpose()
                  << ", Ae/Be/Ce = " << Ae << "/" << Be << "/" << Ce
                  << std::endl;
    }
  }
  return 1;
}