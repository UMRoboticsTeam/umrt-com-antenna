#include "umrt_com_antenna/antenna_math_node.hpp"

/*
 * main:
 *  The following code starts the Antenna PID
 */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<umrt_com_antenna::AntennaPID>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(node);
    executor.spin();
    
    
    rclcpp::shutdown();
    return 0;
}