#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>


using std::placeholders::_1;
using namespace std::chrono_literals;

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd), previous_error(0), integral(0) {}

    double update(double error, double dt) {
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        return output;
    }

private:
    double Kp, Ki, Kd;
    double previous_error, integral;
};


class Patrol : public rclcpp::Node
{
public:
  Patrol()
  : Node("patrol_node"),
  pid_controller(1.5, 0.2, 0.1),
  desired_distance(0.5),
  front_laser(0),
  error(0)
  
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::laser_callback, this, _1));
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
    timer_ = this->create_wall_timer(500ms, std::bind(&Patrol::timer_callback, this));
  }

private:

  PIDController pid_controller;
  float desired_distance;
  float front_laser;
  float error;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {

  
    front_laser = msg->ranges[180];
    float right_laser = msg->ranges[90] - desired_distance;
    float left_laser = msg->ranges[270] - desired_distance;

    error = left_laser - right_laser;

  }

  void timer_callback()
  {
    auto velocity = geometry_msgs::msg::Twist();
    
    if (front_laser > 0.0) {
      // Update the PID controller
      double dt = 0.5; // Update interval (500 ms)
      double angular_velocity = pid_controller.update(error, dt);

      // Move forward
      velocity.linear.x = 0.1;
      velocity.angular.z = angular_velocity;
    }
    else{
    
      velocity.linear.x = 0.1;
      velocity.angular.z = 0.0;
    
    }

    vel_pub->publish(velocity);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}