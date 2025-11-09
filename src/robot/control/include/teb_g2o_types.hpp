#pragma once

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/factory.h>
#include <Eigen/Core>
#include <functional>
#include <cmath>
#include <memory>
#include <typeinfo>

namespace teb_g2o {

template <typename T>
struct Creator : public g2o::AbstractHyperGraphElementCreator {
  g2o::HyperGraph::HyperGraphElement* construct() override { return new T; }
  virtual const std::string& name() const override {
    static std::string n = std::string(typeid(T).name());
    return n;
  }
  virtual ~Creator() {}
};

// Vertex: Pose (x, y, theta)
class VertexPose : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPose() {}
  virtual void setToOriginImpl() override { _estimate.setZero(); }
  virtual void oplusImpl(const double* update) override {
    _estimate += Eigen::Map<const Eigen::Vector3d>(update);
    // normalize theta
    while (_estimate[2] > M_PI) _estimate[2] -= 2*M_PI;
    while (_estimate[2] < -M_PI) _estimate[2] += 2*M_PI;
  }
  virtual bool read(std::istream& in) override { return false; }
  virtual bool write(std::ostream& out) const override { return false; }
};

// Vertex: log(delta_t) between pose i and i+1 (scalar)
class VertexTimeDiff : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexTimeDiff() {}
  virtual void setToOriginImpl() override { _estimate = std::log(0.1); }
  virtual void oplusImpl(const double* update) override {
    _estimate += update[0];
    // no explicit clamp here; exp(_estimate) gives dt>0
    if (_estimate < std::log(1e-4)) _estimate = std::log(1e-4);
    if (_estimate > std::log(100.0)) _estimate = std::log(100.0);
  }
  inline double dt() const { return std::exp(_estimate); }
  virtual bool read(std::istream& in) override { return false; }
  virtual bool write(std::ostream& out) const override { return false; }
};

// Edge: encourage short total time (connects one time-vertex: hinge toward small dt)
class EdgeTimeOptimal : public g2o::BaseUnaryEdge<1, double, VertexTimeDiff> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeTimeOptimal() : weight_(1.0) {}
  void computeError() override {
    double s = _measurement; // desired target on log scale maybe
    const VertexTimeDiff* v = static_cast<const VertexTimeDiff*>(_vertices[0]);
    double s_est = v->estimate();
    _error[0] = weight_ * (s_est - s);
  }
  void setWeight(double w) { weight_ = w; }
  bool read(std::istream& ) override { return false; }
  bool write(std::ostream& ) const override { return false; }
private:
  double weight_;
};

// Edge: shortest-path / path reference for pose (pull pose toward a path point)
class EdgeShortestPath : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeShortestPath() {}
  void computeError() override {
    const VertexPose* v = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d& est = v->estimate();
    _error[0] = est[0] - _measurement[0];
    _error[1] = est[1] - _measurement[1];
  }
  bool read(std::istream& ) override { return false; }
  bool write(std::ostream& ) const override { return false; }
};

// Edge: velocity constraint using (pose_i, pose_{i+1}, dt_i)
// residuals: [v - v_desired, omega - omega_desired] or hinge on limits
class EdgeVelocity : public g2o::BaseMultiEdge<2, Eigen::Vector2d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeVelocity() : max_v_(1.0), max_omega_(1.0) { resize(3); }
  void computeError() override {
    const VertexPose* p1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* p2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt_v = static_cast<const VertexTimeDiff*>(_vertices[2]);
    const Eigen::Vector3d& est1 = p1->estimate();
    const Eigen::Vector3d& est2 = p2->estimate();
    double dx = est2[0] - est1[0];
    double dy = est2[1] - est1[1];
    double dist = std::hypot(dx,dy);
    double dtheta = normalizeAngle(est2[2] - est1[2]);
    double dt = dt_v->dt();
    dt = std::max(dt, 1e-6);
    double v = dist / dt;
    double omega = dtheta / dt;
    // soft residual toward desired velocities (or toward zero for limits)
    _error[0] = (v > max_v_) ? (v - max_v_) : 0.0;
    _error[1] = (std::abs(omega) > max_omega_) ? (std::abs(omega) - max_omega_) : 0.0;
  }
  void setLimits(double vmax, double wmax) { max_v_ = vmax; max_omega_ = wmax; }
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
private:
  double max_v_, max_omega_;
  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }
};

// Edge: holonomic velocity (penalize lateral slip) - binary edge on two poses + time
class EdgeVelocityHolonomic : public g2o::BaseMultiEdge<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeVelocityHolonomic() { resize(3); }
  void computeError() override {
    const VertexPose* p1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* p2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt_v = static_cast<const VertexTimeDiff*>(_vertices[2]);
    const Eigen::Vector3d& est1 = p1->estimate();
    const Eigen::Vector3d& est2 = p2->estimate();
    double dx = est2[0] - est1[0];
    double dy = est2[1] - est1[1];
    // transform into frame of p1
    double c = std::cos(est1[2]), s = std::sin(est1[2]);
    double dy_r = -s*dx + c*dy; // lateral
    double dt = dt_v->dt();
    dt = std::max(dt, 1e-6);
    // penalize lateral component (should be close to 0 for non-holonomic)
    _error[0] = dy_r / dt;
  }
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

// Edge: acceleration (connects two consecutive time-vertices and three poses implicitly)
class EdgeAcceleration : public g2o::BaseMultiEdge<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeAcceleration() : a_max_(1.0) { resize(5); }
  // set distances and dt from outside (graph builder should fill these)
  void computeError() override {
    const VertexPose* p0 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* p1 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* p2 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt_prev = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt_next = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const Eigen::Vector3d& est0 = p0->estimate();
    const Eigen::Vector3d& est1 = p1->estimate();
    const Eigen::Vector3d& est2 = p2->estimate();
    double dx1 = est1[0] - est0[0];
    double dy1 = est1[1] - est0[1];
    double dist_prev = std::hypot(dx1, dy1);
    double dx2 = est2[0] - est1[0];
    double dy2 = est2[1] - est1[1];
    double dist_next = std::hypot(dx2, dy2);
    double prev_dt = dt_prev->dt();
    double next_dt = dt_next->dt();
    double v_prev = dist_prev / std::max(prev_dt, 1e-6);
    double v_next = dist_next / std::max(next_dt, 1e-6);
    double a = (v_next - v_prev) / std::max((prev_dt + next_dt)/2.0, 1e-6);
    _error[0] = std::max(0.0, std::abs(a) - a_max_);
  }
  void setAccelLimit(double amax) { a_max_ = amax; }
private:
  double a_max_;
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

class EdgeForwardVelocity : public g2o::BaseMultiEdge<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeForwardVelocity(double desired_vel = 0.3) : desired_vel_(desired_vel) {
    resize(3);
    _measurement = 0.0;  // not used, but needed
  }

  void computeError() override {
    const VertexPose* p1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* p2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt_v = static_cast<const VertexTimeDiff*>(_vertices[2]);

    double dx = p2->estimate()[0] - p1->estimate()[0];
    double dy = p2->estimate()[1] - p1->estimate()[1];
    double theta = p1->estimate()[2];
    double dt = std::max(dt_v->dt(), 1e-6);

    // Forward displacement in robot frame
    double forward_disp = dx * std::cos(theta) + dy * std::sin(theta);
    double v_forward = forward_disp / dt;

    _error[0] = v_forward - desired_vel_;  // pull toward 0.3 m/s
  }

  void setDesiredVelocity(double v) { desired_vel_ = v; }
  double getDesiredVelocity() const { return desired_vel_; }

  virtual bool read(std::istream&) override { return false; }
  virtual bool write(std::ostream&) const override { return false; }

private:
  double desired_vel_;
};

// Edge: obstacle (pull pose away from obstacles using SDF)
class EdgeObstacle : public g2o::BaseUnaryEdge<1, double, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeObstacle() : inflation_radius_(0.5), weight_(1.0) {}
  void setDistanceAndGradientGetter(std::function<std::pair<double,Eigen::Vector2d>(const Eigen::Vector2d&)> fn) {
    sdf_getter_ = fn;
  }
  void setInflationRadius(double r) { inflation_radius_ = r; }
  void computeError() override {
    const VertexPose* v = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d& p = v->estimate();
    Eigen::Vector2d pos(p[0], p[1]);
    if (!sdf_getter_) { _error[0] = 0.0; return; }
    auto res = sdf_getter_(pos);
    double dist = res.first;
    // residual: positive when inside inflation: (inflation - dist)
    _error[0] = weight_ * std::max(0.0, inflation_radius_ - dist);
    // gradient can be computed in linearization (TODO)
    // if you have gradient res.second (unit vector from obstacle to pose), jacobian wrt pose = -res.second.transpose()
  }
  virtual void linearizeOplus() override {
    const VertexPose* v = static_cast<const VertexPose*>(_vertices[0]);
    const Eigen::Vector3d& p = v->estimate();
    Eigen::Vector2d pos(p[0], p[1]);
    if (!sdf_getter_) { _jacobianOplusXi.setZero(); return; }
    auto res = sdf_getter_(pos);
    double dist = res.first;
    Eigen::Vector2d grad = res.second;
    if (dist >= inflation_radius_) {
      _jacobianOplusXi.setZero();
    } else {
      _jacobianOplusXi(0,0) = weight_ * (-grad[0]);
      _jacobianOplusXi(0,1) = weight_ * (-grad[1]);
      _jacobianOplusXi(0,2) = 0.0;
    }
  }
  void setWeight(double w) { weight_ = w; }
private:
  double inflation_radius_;
  double weight_;
  std::function<std::pair<double,Eigen::Vector2d>(const Eigen::Vector2d&)> sdf_getter_;
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

inline void registerG2OTypes() {
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", std::make_shared<Creator<VertexPose>>());
  factory->registerType("VERTEX_TIMEDIFF", std::make_shared<Creator<VertexTimeDiff>>());
  factory->registerType("EDGE_TIME_OPTIMAL", std::make_shared<Creator<EdgeTimeOptimal>>());
  factory->registerType("EDGE_SHORTEST_PATH", std::make_shared<Creator<EdgeShortestPath>>());
  factory->registerType("EDGE_FORWARD_VELOCITY", std::make_shared<Creator<EdgeForwardVelocity>>());
  factory->registerType("EDGE_VELOCITY", std::make_shared<Creator<EdgeVelocity>>());
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", std::make_shared<Creator<EdgeVelocityHolonomic>>());
  factory->registerType("EDGE_ACCELERATION", std::make_shared<Creator<EdgeAcceleration>>());
  factory->registerType("EDGE_OBSTACLE", std::make_shared<Creator<EdgeObstacle>>());
}

} // namespace teb_g2o