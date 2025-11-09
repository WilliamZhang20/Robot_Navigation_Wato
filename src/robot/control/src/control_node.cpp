#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::TebOptimalPlanner(this->get_logger())) {
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&ControlNode::mapCallback, this, std::placeholders::_1));

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&ControlNode::controlLoop, this));  // Increased from 20ms to 10ms (100Hz)
}

void ControlNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    control_.setOccupancyGrid(msg);
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    control_.setGlobalPath(msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    control_.setOdometry(msg);
}

void ControlNode::controlLoop() {
    control_.optimizeTEB(
        15,      // iterations_innerloop (increased from 10 for better optimization)
        3,       // iterations_outerloop (increased from 2 for more aggressive optimization)
        false,   // compute_cost_afterwards
        1.0,     // obst_cost_scale
        1.0,     // viapoint_cost_scale
        false    // alternative_time_cost
    );
    geometry_msgs::msg::Twist cmd_vel = control_.computeVelocityCommand();

    cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}