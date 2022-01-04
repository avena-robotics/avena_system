#include "arm_controller/hw_interface.hpp"

int main(int argc,char **argv){
    int freq=500;

    rclcpp::init(argc,argv);
    // ArmInterface hw_interface("0AA",freq);
    rclcpp::spin(std::make_shared<ArmInterface>("0AA",freq));
    rclcpp::shutdown();
    return 0;
}