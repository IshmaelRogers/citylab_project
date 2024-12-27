#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_directions.hpp"
#include <memory>

class DirectionService : public rclcpp::Node
{

    public:
        DirectionService() : Node("direction_service")
        {
            service_ = this->create_service<robot_patrol::srv::GetDirections>(
            "/direction_service",
            std::bind(&DirectionService::handle_service, this, std::placeholders::_1, std::placeholders::_2)); 
        }

    private:
        void handle_service(const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
                                std::shared_ptr<robot_patrol::srv::GetDirection::Response> response)
        {

            auto scan = request->laser_data;
            int total_rays = scan.ranges.size();
            int sec_size = total_rays / 3;

            float total_right = 0.0, total_front = 0, total_left = 0;

            for(int i = 0; i < total_rays; ++i)
            {
                if (i < sec_size)
                    total_right += scan.ranges[i];
                else if (i < 2 * sec_size)
                    total_front += scan.ranges[i];
                else
                 total_left += scan.ranges[i];
            }

            if (total_front > 0.35 * sec_size)
            {
                response->direction = "forward";
            }
            else if (total_left > total_right)
            {
                response->direction = "left";
            }
            else 
            {
                response->direction = "right";
            }

            RCLCPP_INFO(this->get_logger(), "Direction: %s", response->direction.c_str());                        
        }

        rclcpp::Service<robot_patrol::srv::GetDirections>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();
    return 0;


}