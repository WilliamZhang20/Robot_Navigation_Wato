#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include "control_core.hpp"
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <utility>
#include <geometry_msgs/msg/twist.hpp>
#include <g2o/core/optimizable_graph.h>

using namespace robot;
using namespace teb_g2o;

TebOptimalPlanner::TebOptimalPlanner(const rclcpp::Logger& logger) : logger_(logger), optimized_(false) {
  // default config tweaks if desired
  TebOptimalPlanner::registerG2OTypes();

  // prepare optimizer
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > BlockSolverType;
  typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
  auto linearSolver = std::make_unique<LinearSolverType>();
  auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
  auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
  optimizer_.reset(new g2o::SparseOptimizer());
  optimizer_->setAlgorithm(algorithm);
  optimizer_->setVerbose(false);
}

TebOptimalPlanner::~TebOptimalPlanner() {
  clearGraph();
  optimizer_.reset();
}

void TebOptimalPlanner::registerG2OTypes() {
  teb_g2o::registerG2OTypes();
}

void TebOptimalPlanner::setGlobalPath(const nav_msgs::msg::Path::SharedPtr &path) {
  current_path_ = path;
}

void TebOptimalPlanner::setOdometry(const nav_msgs::msg::Odometry::SharedPtr &odom) {
  current_odom_ = odom;
}

void TebOptimalPlanner::setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr &map) {
  current_map_ = map;
  computeDistanceMapFromGrid();
}

void TebOptimalPlanner::computeDistanceMapFromGrid() {
  if (!current_map_) {
    distance_map_.clear();
    return;
  }

  int width = current_map_->info.width;
  int height = current_map_->info.height;
  double resolution = current_map_->info.resolution;
  
  distance_map_.resize(height, std::vector<double>(width, std::numeric_limits<double>::infinity()));
  
  // Queue for BFS-based Euclidean Distance Transform
  std::queue<std::pair<int, int>> q;
  
  // Mark obstacle cells and initialize queue
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = y * width + x;
      int8_t val = current_map_->data[idx];
      
      // Occupied cells (val > 50) or unknown cells (val < 0) are obstacles
      if (val > 50 || val < 0) {
        distance_map_[y][x] = 0.0;
        q.push({x, y});
      }
    }
  }
  
  // BFS to propagate distances
  while (!q.empty()) {
    auto [cx, cy] = q.front();
    q.pop();
    
    // Check 8-connected neighbors
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        
        int nx = cx + dx;
        int ny = cy + dy;
        
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        
        double diag_dist = std::sqrt(dx*dx + dy*dy) * resolution;
        double new_dist = distance_map_[cy][cx] + diag_dist;
        
        if (new_dist < distance_map_[ny][nx]) {
          distance_map_[ny][nx] = new_dist;
          q.push({nx, ny});
        }
      }
    }
  }
}

// small helpers
double TebOptimalPlanner::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::pair<double,double> TebOptimalPlanner::nearestPointOnPath(double x, double y) {
  // simple brute force projection onto segments (similar to your previous code)
  double best_d = std::numeric_limits<double>::infinity();
  double bx = x, by = y;
  if (!current_path_ || current_path_->poses.size() < 2) return {bx, by};
  for (size_t i=0; i+1<current_path_->poses.size(); ++i) {
    double x1 = current_path_->poses[i].pose.position.x;
    double y1 = current_path_->poses[i].pose.position.y;
    double x2 = current_path_->poses[i+1].pose.position.x;
    double y2 = current_path_->poses[i+1].pose.position.y;
    double dx = x2-x1, dy = y2-y1;
    double l2 = dx*dx + dy*dy;
    double t = (l2>1e-9) ? ((x-x1)*dx + (y-y1)*dy)/l2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    double px = x1 + t*dx, py = y1 + t*dy;
    double d = std::hypot(px-x, py-y);
    if (d < best_d) { best_d = d; bx = px; by = py; }
  }
  return {bx, by};
}

void TebOptimalPlanner::autoResize(double dt_ref, double dt_hyst, int min_samples, int max_samples, bool fast_mode) {
  // trivial autosizing: set sample count so horizon ~ dt_ref * samples
  double horizon = 3.0; // you can base on cfg or env
  int suggested = std::clamp(static_cast<int>(std::round(horizon / dt_ref)), min_samples, max_samples);
  // If this affects teb_ vector length, resize and distribute times
  if (teb_.size() > (size_t)suggested) teb_.resize(suggested);
  else if (teb_.size() < (size_t)suggested) {
    // duplicate last pose to extend
    if (!teb_.empty()) {
      TrajPose last = teb_.back();
      while (teb_.size() < (size_t)suggested) {
        last.t += dt_ref;
        teb_.push_back(last);
      }
    }
  }
}

bool TebOptimalPlanner::buildGraph(double weight_multiplier) {
  clearGraph();
  if (teb_.size() < 2) return false;

  optimizer_->clear();
  pose_vertices_.clear();
  time_vertices_.clear();

  // create pose vertices
  int vid = 0;
  for (size_t i=0;i<teb_.size();++i) {
    auto v = new VertexPose();
    v->setId(vid++);
    Eigen::Vector3d est(teb_[i].x, teb_[i].y, teb_[i].theta);
    v->setEstimate(est);
    if (i==0) v->setFixed(true); // fix first pose to robot pose
    optimizer_->addVertex(v);
    pose_vertices_.push_back(v);
  }

  // create time vertices (N-1), parameterized as log(dt)
  for (size_t i=0;i+1<teb_.size();++i) {
    auto vt = new VertexTimeDiff();
    vt->setId(vid++);
    double dt_init = std::max(1e-3, teb_[i+1].t - teb_[i].t);
    vt->setEstimate(std::log(dt_init));
    optimizer_->addVertex(vt);
    time_vertices_.push_back(vt);
  }

  // edges: shortest path reference for each pose (except first maybe)
  int eid = 0;
  for (size_t i=1;i<pose_vertices_.size();++i) {
    auto ref = nearestPointOnPath(teb_[i].x, teb_[i].y);
    auto e = new EdgeShortestPath();
    e->setId(eid++);
    e->setVertex(0, pose_vertices_[i]);
    e->setMeasurement(Eigen::Vector2d(ref.first, ref.second));
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * (5.0 * weight_multiplier);
    e->setInformation(info);
    optimizer_->addEdge(e);
  }

  // velocity edges linking (pose_i, pose_{i+1}) and dt_i
  for (size_t i=0;i+1<pose_vertices_.size();++i) {
    auto e = new EdgeVelocity();
    e->setId(eid++);
    e->setVertex(0, pose_vertices_[i]);
    e->setVertex(1, pose_vertices_[i+1]);
    e->setVertex(2, time_vertices_[i]);
    e->setLimits(cfg_.robot.max_vel_x, cfg_.robot.max_vel_theta);
    Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * (2.0 * weight_multiplier);
    e->setInformation(info);
    optimizer_->addEdge(e);
  }

  // holonomic lateral slip edges (penalize lateral displacement)
  for (size_t i=0;i+1<pose_vertices_.size();++i) {
    auto eh = new EdgeVelocityHolonomic();
    eh->setId(eid++);
    eh->setVertex(0, pose_vertices_[i]);
    eh->setVertex(1, pose_vertices_[i+1]);
    eh->setVertex(2, time_vertices_[i]);
    Eigen::Matrix<double,1,1> info;
    info(0,0) = 10.0 * weight_multiplier;
    eh->setInformation(info);
    optimizer_->addEdge(eh);
  }

  // acceleration edges (simple hinge on successive dt's)
  for (size_t i=0;i+2<pose_vertices_.size();++i) {
    auto ea = new EdgeAcceleration();
    ea->setId(eid++);
    ea->setVertex(0, pose_vertices_[i]);
    ea->setVertex(1, pose_vertices_[i+1]);
    ea->setVertex(2, pose_vertices_[i+2]);
    ea->setVertex(3, time_vertices_[i]);
    ea->setVertex(4, time_vertices_[i+1]);
    ea->setAccelLimit(cfg_.robot.acc_lim_x);
    Eigen::Matrix<double,1,1> info;
    info(0,0) = 5.0 * weight_multiplier;
    ea->setInformation(info);
    optimizer_->addEdge(ea);
  }

  // obstacle edges - attach to every pose (except maybe first)
  auto sdf_getter = [this](const Eigen::Vector2d& p) -> std::pair<double, Eigen::Vector2d> {
    if (distance_map_.empty() || !current_map_) return {1e6, {0,0}};
    // convert world to map coords
    double resolution = current_map_->info.resolution;
    double ox = current_map_->info.origin.position.x;
    double oy = current_map_->info.origin.position.y;
    double mx = (p[0] - ox) / resolution;
    double my = (p[1] - oy) / resolution;
    int ix = static_cast<int>(std::round(mx));
    int iy = static_cast<int>(std::round(my));
    int width = current_map_->info.width;
    int height = current_map_->info.height;
    if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
      return {1e6, {0,0}};
    }
    double dist = distance_map_[iy][ix];
    // approximate gradient using finite differences
    Eigen::Vector2d grad(0.0, 0.0);
    if (dist > 0.0) {
      // x direction
      int ix_p = ix + 1, ix_m = ix - 1;
      double d_p = (ix_p >= 0 && ix_p < width) ? distance_map_[iy][ix_p] : dist;
      double d_m = (ix_m >= 0 && ix_m < width) ? distance_map_[iy][ix_m] : dist;
      grad[0] = (d_p - d_m) / (2.0 * resolution);
      // y direction
      int iy_p = iy + 1, iy_m = iy - 1;
      d_p = (iy_p >= 0 && iy_p < height) ? distance_map_[iy_p][ix] : dist;
      d_m = (iy_m >= 0 && iy_m < height) ? distance_map_[iy_m][ix] : dist;
      grad[1] = (d_p - d_m) / (2.0 * resolution);
      double norm = grad.norm();
      if (norm > 1e-6) {
        grad /= norm;
      } else {
        grad.setZero();
      }
    }
    return {dist, grad};
  };

  for (size_t i=1;i<pose_vertices_.size();++i) {
    auto eo = new EdgeObstacle();
    eo->setId(eid++);
    eo->setVertex(0, pose_vertices_[i]);
    eo->setInflationRadius(0.5);
    eo->setDistanceAndGradientGetter(sdf_getter);
    eo->setWeight(100.0 * weight_multiplier);
    Eigen::Matrix<double,1,1> info;
    info(0,0) = 1.0 * weight_multiplier;
    eo->setInformation(info);
    optimizer_->addEdge(eo);
  }

  // optional: time optimal edges (encourage small dt) on each time vertex
  for (size_t i=0;i<time_vertices_.size(); ++i) {
    auto et = new EdgeTimeOptimal();
    et->setId(eid++);
    et->setVertex(0, time_vertices_[i]);
    double s_measure = std::log(cfg_.trajectory.dt_ref); // desire log(dt_ref)
    et->setMeasurement(s_measure);
    et->setWeight(0.05 * weight_multiplier); // small push toward dt_ref
    Eigen::Matrix<double,1,1> info;
    info(0,0) = 1e-2 * weight_multiplier;
    et->setInformation(info);
    optimizer_->addEdge(et);
  }

  return true;
}

bool TebOptimalPlanner::optimizeGraph(int iterations, bool verbose) {
  if (!optimizer_) return false;
  optimizer_->initializeOptimization();
  optimizer_->setVerbose(verbose);
  int it = optimizer_->optimize(iterations);
  (void)it;
  // read back results
  // update teb_ poses and times
  for (size_t i=0;i<pose_vertices_.size() && i<teb_.size(); ++i) {
    Eigen::Vector3d est = pose_vertices_[i]->estimate();
    teb_[i].x = est[0];
    teb_[i].y = est[1];
    teb_[i].theta = est[2];
  }
  // recompute absolute times based on optimized dt
  if (!teb_.empty()) {
    teb_[0].t = 0.0;
    for (size_t i=0;i<time_vertices_.size(); ++i) {
      teb_[i+1].t = teb_[i].t + time_vertices_[i]->dt();
    }
  }
  return true;
}

void TebOptimalPlanner::clearGraph() {
  // optimizer_->clear() may be sufficient but also cleanup stored pointers
  if (optimizer_) optimizer_->clear();
  pose_vertices_.clear();
  time_vertices_.clear();
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost) {
  (void)alternative_time_cost;
  // compute and store a cost vector if desired (sum residuals)
  // For demonstration, compute simple sums
  double sum_obs = 0.0, sum_time = 0.0, sum_vel = 0.0;
  // iterate edges in optimizer and sum squared errors
  if (!optimizer_) return;
  optimizer_->computeActiveErrors();
  for (auto* e : optimizer_->edges()) {
    g2o::OptimizableGraph::Edge* edge = dynamic_cast<g2o::OptimizableGraph::Edge*>(e);
    if (edge) {
      double chi = edge->chi2();
      if (dynamic_cast<EdgeObstacle*>(edge)) sum_obs += chi * obst_cost_scale;
      else if (dynamic_cast<EdgeShortestPath*>(edge)) sum_obs += chi * viapoint_cost_scale;
      else if (dynamic_cast<EdgeTimeOptimal*>(edge)) sum_time += chi;
      else if (dynamic_cast<EdgeVelocity*>(edge) || dynamic_cast<EdgeVelocityHolonomic*>(edge) || dynamic_cast<EdgeAcceleration*>(edge)) sum_vel += chi;
      else sum_obs += chi;
    }
  }
  // You can publish or store these values
  (void)sum_obs; (void)sum_time; (void)sum_vel;
}

geometry_msgs::msg::Twist TebOptimalPlanner::computeVelocityCommand() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  if (!optimized_ || teb_.size() < 2) {
    return cmd;
  }

  // Extract velocity from first trajectory segment
  double dx = teb_[1].x - teb_[0].x;
  double dy = teb_[1].y - teb_[0].y;
  double dtheta = teb_[1].theta - teb_[0].theta;
  
  // Normalize angle difference
  while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
  while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
  
  double dt = std::max(1e-3, teb_[1].t - teb_[0].t);
  
  // Compute velocity in global frame
  double vx_global = dx / dt;
  double vy_global = dy / dt;
  double omega = dtheta / dt;
  
  // Transform to robot frame using current orientation
  double cos_theta = std::cos(teb_[0].theta);
  double sin_theta = std::sin(teb_[0].theta);
  
  cmd.linear.x = vx_global * cos_theta + vy_global * sin_theta;
  cmd.linear.y = -vx_global * sin_theta + vy_global * cos_theta;
  cmd.angular.z = omega;
  
  // Apply velocity limits if configured
  if (cfg_.robot.max_vel_x > 0.0) {
    cmd.linear.x = std::clamp(cmd.linear.x, -cfg_.robot.max_vel_x, cfg_.robot.max_vel_x);
  }
  if (cfg_.robot.max_vel_y > 0.0) {
    cmd.linear.y = std::clamp(cmd.linear.y, -cfg_.robot.max_vel_y, cfg_.robot.max_vel_y);
  }
  if (cfg_.robot.max_vel_theta > 0.0) {
    cmd.angular.z = std::clamp(cmd.angular.z, -cfg_.robot.max_vel_theta, cfg_.robot.max_vel_theta);
  }

  return cmd;
}

bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost) {
  if (cfg_.optim.optimization_activate == false) return false;
  bool success = false;
  optimized_ = false;
  double weight_multiplier = 1.0;
  bool fast_mode = !cfg_.obstacles.include_dynamic_obstacles;

  for (int i=0; i<iterations_outerloop; ++i) {
    if (cfg_.trajectory.teb_autosize) {
      autoResize(cfg_.trajectory.dt_ref, cfg_.trajectory.dt_hysteresis, cfg_.trajectory.min_samples, cfg_.trajectory.max_samples, fast_mode);
    }
    // build internal band from path/odom if not present
    if (teb_.empty()) {
      // simple initialization using path: sample up to N poses or build N repeated last
      if (!current_odom_ || !current_path_) return false;
      double rx = current_odom_->pose.pose.position.x;
      double ry = current_odom_->pose.pose.position.y;
      double rtheta = extractYaw(current_odom_->pose.pose.orientation);
      int N = 15;
      teb_.clear();
      TrajPose p0{rx, ry, rtheta, 0.0};
      teb_.push_back(p0);
      double lastx=rx, lasty=ry;
      double horizon = 3.0;
      double desired = 0.5;
      double time_per_segment = horizon / (desired * (N-1));
      size_t sampled = 1;
      for (size_t k=0; k<current_path_->poses.size() && sampled < (size_t)N; ++k) {
        double px = current_path_->poses[k].pose.position.x;
        double py = current_path_->poses[k].pose.position.y;
        double seg = std::hypot(px-lastx, py-lasty);
        if (seg < 1e-3) continue;
        double theta = std::atan2(py-lasty, px-lastx);
        TrajPose tp{px, py, theta, time_per_segment * sampled};
        teb_.push_back(tp);
        lastx = px; lasty = py;
        sampled++;
      }
      while (teb_.size() < (size_t)N) {
        TrajPose last = teb_.back();
        last.t += time_per_segment;
        teb_.push_back(last);
      }
    }

    success = buildGraph(weight_multiplier);
    if (!success) { clearGraph(); return false; }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success) { clearGraph(); return false; }
    optimized_ = true;

    if (compute_cost_afterwards && i==iterations_outerloop-1) {
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
    }
    clearGraph();
    weight_multiplier *= cfg_.optim.weight_adapt_factor;
  }

  return true;
}