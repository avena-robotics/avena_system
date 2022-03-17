#include "arm_controller/hw_interface.hpp"

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    std::shared_ptr<ArmInterface> hw_interface;
    hw_interface = std::make_shared<ArmInterface>();
    rclcpp::spin(hw_interface);
    // rclcpp::spin(std::make_shared<ArmInterface>("0AA"));
    return 0;
}