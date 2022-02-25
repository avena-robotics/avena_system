#include "arm_controller/hw_interface.hpp"

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    ArmInterface hw_interface("000");
    // rclcpp::spin(std::make_shared<ArmInterface>("0AA"));
    return 0;
}