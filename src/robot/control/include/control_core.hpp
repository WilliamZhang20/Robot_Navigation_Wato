#pragma once

#include "teb_g2o_types.hpp"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robot {

struct TrajPose {
  double x, y, theta;
  double t; // absolute time from start or incremental
};

struct TebConfig {
  struct Optim { bool optimization_activate = true; double weight_adapt_factor = 1.4; } optim;
  struct Obs { bool include_dynamic_obstacles = false; } obstacles;
  struct Traj { bool teb_autosize = false; double dt_ref = 0.2; double dt_hysteresis = 0.05;
                int min_samples = 3; int max_samples = 50; } trajectory;
  struct Robot { double max_vel_x = 0.4; double max_vel_y = 0.2; double max_vel_theta = 1.0; double acc_lim_x = 1.0; } robot;
};

class TebOptimalPlanner {
public:
  TebOptimalPlanner(const rclcpp::Logger& logger);
  ~TebOptimalPlanner();

  // register g2o creator types
  static void registerG2OTypes();

  // main optimize call
  bool optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                   double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost);

  // external setters (path, odom)
  void setGlobalPath(const nav_msgs::msg::Path::SharedPtr &path);
  void setOdometry(const nav_msgs::msg::Odometry::SharedPtr &odom);
  void setOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr &map);
  geometry_msgs::msg::Twist computeVelocityCommand();

  // access optimized TEB
  const std::vector<TrajPose>& getOptimizedBand() const { return teb_; }

private:
  rclcpp::Logger logger_;
  // helper steps
  bool buildGraph(double weight_multiplier);
  bool optimizeGraph(int iterations, bool verbose);
  void clearGraph();
  void computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost);

  // auto-resize trajectory (simple heuristic)
  void autoResize(double dt_ref, double dt_hyst, int min_samples, int max_samples, bool fast_mode);

  // helpers
  double extractYaw(const geometry_msgs::msg::Quaternion& q);
  std::pair<double,double> nearestPointOnPath(double x, double y);
  // SDF / distance map helper
  void computeDistanceMapFromGrid();

  // members
  TebConfig cfg_;
  std::vector<TrajPose> teb_;
  nav_msgs::msg::Path::SharedPtr current_path_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  std::vector<std::vector<double>> distance_map_;

  // g2o
  std::unique_ptr<g2o::SparseOptimizer> optimizer_;

  // keep mapping of created vertices for reading back after optimization
  std::vector<teb_g2o::VertexPose*> pose_vertices_;
  std::vector<teb_g2o::VertexTimeDiff*> time_vertices_;

  bool optimized_;
};

} // namespace robot