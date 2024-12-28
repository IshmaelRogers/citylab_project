#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

constexpr double OBSTACLE_THRESHOLD = 0.35;  // 35 cm threshold for obstacles
constexpr double SAFE_THRESHOLD = 1.0;       // Threshold for safe distances

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::analyze_laser_data, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Service Server Ready.");
  }

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;

  void analyze_laser_data(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {
    
    auto scan = request->laser_data;
    int length = scan.ranges.size();
    double angle_min = scan.angle_min;
    double angle_max = scan.angle_max;

    // Function to map angle to laser scan index
    auto transform_index = [&](double angle) {
      return static_cast<int>((length * (angle - angle_min)) /
                              (angle_max - angle_min));
    };

    int start_left = transform_index(M_PI / 6);
    int end_left = transform_index(M_PI / 2);

    int start_front = transform_index(-M_PI / 6);
    int end_front = transform_index(M_PI / 6);

    int start_right = transform_index(-M_PI / 2);
    int end_right = transform_index(-M_PI / 6);

    double total_left = 0.0, total_front = 0.0, total_right = 0.0;

    // Sum laser distances in each section
    for (int i = start_left; i <= end_left; ++i) {
      total_left += scan.ranges[i];
    }
    for (int i = start_front; i <= end_front; ++i) {
      total_front += scan.ranges[i];
    }
    for (int i = start_right; i <= end_right; ++i) {
      total_right += scan.ranges[i];
    }

    // Decision Logic
    if (total_front / (end_front - start_front) > OBSTACLE_THRESHOLD) {
      response->direction = "forward";
    } else {
      if (total_left > total_right) {
        response->direction = "left";
      } else {
        response->direction = "right";
      }
    }

    RCLCPP_INFO(this->get_logger(), "Service Completed. Direction: %s",
                response->direction.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
