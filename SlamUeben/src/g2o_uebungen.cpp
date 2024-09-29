#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <random>
#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>

class PlaneFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }

  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  virtual bool read(std::istream &in) {}
  virtual bool write(std::ostream &out) const {}
};

class PlaneFittingEdge
    : public g2o::BaseUnaryEdge<1, double, PlaneFittingVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlaneFittingEdge(double x, double y) : BaseUnaryEdge(), x_(x), y_(y) {}

  virtual void computeError() override {
    const PlaneFittingVertex *v =
        static_cast<const PlaneFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0) = _measurement - (abc(0) * x_ + abc(1) * y_ + abc(2));
  }

  virtual void linearizeOplus() override {
    const PlaneFittingVertex *v =
        static_cast<const PlaneFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _jacobianOplusXi[0] = -x_;
    _jacobianOplusXi[1] = -y_;
    _jacobianOplusXi[2] = -1;
  }

  virtual bool read(std::istream &in) {}
  virtual bool write(std::ostream &out) const {}

 public:
  double x_, y_;
};

typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
    linearSolverType;

int main(int argc, char **argv) {
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

  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<BlockSolverType>(std::make_unique<linearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  PlaneFittingVertex *v = new PlaneFittingVertex();
  double init_value = 10;
  v->setEstimate(Eigen::Vector3d(init_value, init_value, init_value));
  v->setId(0);
  optimizer.addVertex(v);

  for(int i = 0; i < xs.size(); i++) {
    PlaneFittingEdge *edge = new PlaneFittingEdge(xs[i], ys[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(zs[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / 0.01);
    optimizer.addEdge(edge);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);
  auto abc_est = v->estimate();
  std::cout << "abc_est = " << abc_est.transpose() << std::endl;
  return 1;
}