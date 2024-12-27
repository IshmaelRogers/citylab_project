#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_directions.hpp"

class DirectionService : public rclcpp::Node
{

    public:
        DirectionService() : Node("direction_service")
        {
            service_ = this->create_service<robot_patrol::srv::GetDirections>(
            "/direction_service",
            std::bind(&DirectionService::handle_service, this, std::placeholders::_1, std::placeholders::_2)); 
        }

        


};

int main(int argc, char **argv)
{
    return 0;


}