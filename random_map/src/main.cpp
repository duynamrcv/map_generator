#include <rclcpp/rclcpp.hpp>
#include "random_map/RandomMapSensing.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomMapSensing>());
    rclcpp::shutdown();
    return 0;
}