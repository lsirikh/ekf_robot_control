#include "ekf_robot_control/interface.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Interface>(true, false);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
