#include <rclcpp/rclcpp.hpp>
#include "motion_planning/KinoAStar.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinoAStar>());
    rclcpp::shutdown();
    return 0;
}