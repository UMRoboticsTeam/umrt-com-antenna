#include "umrt_com_antenna/antenna_pid.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AntennaPID>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(node);
    executor.spin();
    
    
    rclcpp::shutdown();
    return 0;
}