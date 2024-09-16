#include <rclcpp/rclcpp.hpp>
#include "grid_map/GridMap.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridMap>());
    rclcpp::shutdown();
    return 0;
}