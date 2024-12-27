#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespacestd::chrono_literals;

constexpr double linear_speed = 0.1;
constexpr double obstacle_limit = 0.35;

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol"), direction_(0.0)
    {
        // Subscriber to LaserScan
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));

        // Publisher to cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer for control loop at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::control_loop, this));
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int center_index = msg->ranges.size() / 2;
        int half_range = msg->ranges.size() / 4;

        float max_distance = 0.0;
        int best_index = center_index;  // Default to moving straight if no clear path

        // Scan the front 180 degrees
        for (int i = center_index - half_range; i <= center_index + half_range; ++i)
        {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_distance && msg->ranges[i] > 0.35)
            {
                max_distance = msg->ranges[i];
                best_index = i;
            }
        }

        if (max_distance < 0.5) {
            // Stop if obstacles are too close
            direction_ = 0.0;
        } else {
            float angle = msg->angle_min + best_index * msg->angle_increment;
            direction_ = angle;
        }
    }

    void control_loop()
    {
        auto cmd = geometry_msgs::msg::Twist();
        if (direction_ == 0.0) {
            // Rotate in place if no clear path is found
            cmd.angular.z = 0.5;
        } else {
            cmd.linear.x = 0.1;  // Constant forward speed
            cmd.angular.z = std::clamp(direction_ / 2, -0.5, 0.5);  // Smooth turning
        }
        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float direction_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}
