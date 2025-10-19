#include <rclcpp/rclcpp.hpp>
#include "rosbag_recorder/rosbag_recorder_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<rosbag_recorder::RosbagRecorderNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rosbag_recorder_main"), 
                 "Exception in main: %s", e.what());
    return 1;
  }
  
  return 0;
}
