#include <rclcpp/rclcpp.hpp>
#include "map_generator/RandomMapSensing.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomMapSensing>());
    rclcpp::shutdown();
    return 0;
}