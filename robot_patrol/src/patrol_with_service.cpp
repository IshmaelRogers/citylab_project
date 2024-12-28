#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

constexpr double OBSTACLE_LIMIT = 0.35;
constexpr double LINEAR_SPEED = 0.1;
constexpr double TURN_SPEED = 0.5;

int obstacle_count_threshold = 10;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_with_service_node") {
    // Create callback groups
    timer_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    laser_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = laser_cb_group_;

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laser_scan_callback, this, std::placeholders::_1),
        subscription_options);
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_cb_group_);
    
    client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");

    RCLCPP_INFO(this->get_logger(), "Patrol Commander Created.");
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void timer_callback();
  void call_direction_service(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);
  bool check_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr laser_cb_group_;

  bool front_obstacle = false;
  bool request_made = false;
  bool turning_ = false;
  std::string direction_ = "forward";
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

void Patrol::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_message) {
  front_obstacle = false;
  last_scan_ = scan_message;

  // Transform radian angle to indices
  int length = scan_message->ranges.size();
  double angle_min = scan_message->angle_min;
  double angle_max = scan_message->angle_max;

  auto transform_index = [&](double angle) {
    return static_cast<int>((length * (angle - angle_min)) /
                            (angle_max - angle_min));
  };

  int start_front = transform_index(-M_PI / 6);
  int end_front = transform_index(M_PI / 6);

  int front_count = 0;
  for (int i = start_front; i <= end_front; ++i) {
    if (scan_message->ranges[i] < OBSTACLE_LIMIT) {
      front_count++;
    }
  }

  if (front_count >= obstacle_count_threshold) {
    front_obstacle = true;
  }
}

bool Patrol::check_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  int length = scan_msg->ranges.size();
  double angle_min = scan_msg->angle_min;
  double angle_max = scan_msg->angle_max;

  auto transform_index = [&](double angle) {
    return static_cast<int>((length * (angle - angle_min)) /
                            (angle_max - angle_min));
  };

  int start_front = transform_index(-M_PI / 6);
  int end_front = transform_index(M_PI / 6);

  for (int i = start_front; i <= end_front; ++i) {
    if (scan_msg->ranges[i] < OBSTACLE_LIMIT) {
      return false;  // Path is still blocked
    }
  }

  return true;  // Path is clear
}

void Patrol::timer_callback() {
  auto command = geometry_msgs::msg::Twist();

  // If currently turning, check if the path is clear to stop turning
  if (turning_) {
    if (check_path_clear(last_scan_)) {
      turning_ = false;
      RCLCPP_INFO(this->get_logger(), "Path cleared. Resuming forward motion.");
    } else {
      // Continue turning
      command.linear.x = 0.0;
      command.angular.z = (direction_ == "left") ? TURN_SPEED : -TURN_SPEED;
      publisher_->publish(command);
      return;
    }
  }

  // Default forward movement
  command.linear.x = LINEAR_SPEED;
  command.angular.z = 0.0;

  if (!front_obstacle) {
    publisher_->publish(command);
    return;
  }

  // Call service if obstacle is detected and no request is in progress
  if (front_obstacle && !request_made) {
    call_direction_service(last_scan_);
  }

  // Stop and turn based on direction
  if (direction_ == "left") {
    turning_ = true;
    command.linear.x = 0.0;
    command.angular.z = TURN_SPEED;
    RCLCPP_INFO(this->get_logger(), "Turning left...");
  } else if (direction_ == "right") {
    turning_ = true;
    command.linear.x = 0.0;
    command.angular.z = -TURN_SPEED;
    RCLCPP_INFO(this->get_logger(), "Turning right...");
  }

  publisher_->publish(command);
}

void Patrol::call_direction_service(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (request_made) {
    RCLCPP_WARN(this->get_logger(), "Waiting for last request to complete");
    return;
  }
  
  auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
  request->laser_data = *msg;

  request_made = true;
  auto request_future = client_->async_send_request(
      request,
      std::bind(&Patrol::client_response_callback, this, std::placeholders::_1));
}

void Patrol::client_response_callback(
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
  auto result = future.get();
  direction_ = result->direction;
  request_made = false;
  RCLCPP_INFO(this->get_logger(), "Service Response: %s", direction_.c_str());
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
