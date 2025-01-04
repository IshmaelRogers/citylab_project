#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "robot_patrol/action/go_to_pose.hpp"  // Generated from GoToPose.action

using namespace std::chrono_literals;

struct Pose2D
{
  double x;
  double y;
  double theta;
};

class GoToPose : public rclcpp::Node
{
public:
  // Define the Action type using the generated C++ class
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose()
  : Node("go_to_pose_action_node")
  {
    // Initialize desired and current positions
    desired_pos_ = {0.0, 0.0, 0.0};
    current_pos_ = {0.0, 0.0, 0.0};

    // Subscriber to /odom
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(&GoToPose::odomCallback, this, std::placeholders::_1)
    );

    // Publisher to /cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create the action server
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
      this,
      "go_to_pose",
      std::bind(&GoToPose::handleGoal, this, _1, _2),
      std::bind(&GoToPose::handleCancel, this, _1),
      std::bind(&GoToPose::handleAccepted, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "GoToPose Action Server has been started.");
  }

private:
  // ------------------
  // ACTION SERVER PART
  // ------------------
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;

  // Called upon receiving a new goal request
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToPoseAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request x=%.2f, y=%.2f, theta=%.2f",
      goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

    // Decide whether to accept or reject the goal
    // For this example, accept all goals
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Called if the client requests to cancel
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle)
  {
    RCLCPP_WARN(this->get_logger(), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Called once the goal is accepted
  void handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
  {
    // We spin up a new thread here so that execution doesn’t block the server
    std::thread{std::bind(&GoToPose::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // Execution of the goal (control loop)
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Starting execution of goal...");

    // Extract goal
    auto goal = goal_handle->get_goal();
    desired_pos_.x = goal->goal_pos.x;
    desired_pos_.y = goal->goal_pos.y;
    desired_pos_.theta = goal->goal_pos.theta;  // degrees

    // Feedback object
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();

    // Result object
    auto result = std::make_shared<GoToPoseAction::Result>();
    result->status = false;

    rclcpp::Rate rate(10.0);  // 10 Hz
    double distance_tolerance = 0.05;  // 5 cm
    double angle_tolerance_deg = 5.0;  // degrees

    while (rclcpp::ok())
    {
      // Check if goal is canceled
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "Goal canceled.");
        result->status = false;
        goal_handle->canceled(result);
        return;
      }

      // Publish feedback: current robot pose
      feedback->current_pos.x = current_pos_.x;
      feedback->current_pos.y = current_pos_.y;
      feedback->current_pos.theta = current_pos_.theta;
      goal_handle->publish_feedback(feedback);

      // Compute difference
      double dx = desired_pos_.x - current_pos_.x;
      double dy = desired_pos_.y - current_pos_.y;

      // Distance to goal
      double distance = std::sqrt(dx*dx + dy*dy);

      // Convert current_pos_.theta (degrees) to radians for heading
      double current_theta_rad = current_pos_.theta * M_PI / 180.0;
      double desired_theta_rad = desired_pos_.theta * M_PI / 180.0;

      // Orientation difference
      double dtheta = desired_theta_rad - current_theta_rad;
      // Normalize angle to range [-pi, pi]
      dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta));

      // Check if we have reached the goal
      if (distance < distance_tolerance && std::fabs(dtheta) < (angle_tolerance_deg * M_PI / 180.0))
      {
        // Stop the robot
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);

        RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
        result->status = true;
        goal_handle->succeed(result);
        return;
      }

      // ----------------
      // CONTROL STRATEGY
      // ----------------
      // Let’s keep it very simple:
      // 1) Always go forward at 0.2 m/s
      // 2) Turn to reduce angular error
      geometry_msgs::msg::Twist cmd_vel;

      cmd_vel.linear.x = 0.2;  // constant linear speed

      // Proportional control for rotation
      double kp = 0.5;  // tune as needed
      cmd_vel.angular.z = kp * dtheta;

      // Publish velocity command
      cmd_vel_pub_->publish(cmd_vel);

      rate.sleep();
    }
  }

  // --------------------------------
  // ODOMETRY & POSITION UPDATES
  // --------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extract x, y
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // Extract orientation (quaternion -> Euler)
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert yaw in radians to degrees
    current_pos_.theta = yaw * 180.0 / M_PI;
  }

  // ------------------
  // CLASS VARIABLES
  // ------------------
  Pose2D desired_pos_;
  Pose2D current_pos_;

  // Subscriber / Publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
