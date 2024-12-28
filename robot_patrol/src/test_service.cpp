#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;


class TestService : public rclcpp::Node
{
public:
    TestService() : Node("test_service_node")
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&TestService::laser_callback, this, std::placeholders::_1));

        client_ = this->create_client<robot_patrol::srv::GetDirection>("direction_service");
        RCLCPP_INFO(this->get_logger(), "Service Client Ready");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);
};

void TestService::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        while(!client_->wait_for_service(1s)){
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service Interrupted. Exiting now!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Direction service not available, waiting again...");
    }
        request->laser_data = *msg;
        auto result_future = client_->async_send_request(
             request, std::bind(&TestService::client_response_callback, this,
             std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Service Requedt: Sent");
}

void TestService::client_response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture result_future)
{
    auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Service Response: Direction -> %s", result->direction.c_str());

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto test_node = std::make_shared<TestService>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}