// ROS 2 Listener in C++
#include <iostream>
#include <memory>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// Need to change the header files and use C++ 11 builtins

int times = 0;
void chatterCallback(const std_msgs::msg::String::SharedPtr msg){
  std::cout << "I heard: [" << msg->data << "]" << std::endl;
  times++;

  if (times == 10) {
    rclcpp::shutdown(); //nullptr); //, "Done listening.");
  }
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv); // Initialization and create the object. Callback is using QoS profiles
  auto node = rclcpp::Node::make_shared("listener");
  auto sub =
    node->create_subscription<std_msgs::msg::String>(
                                                     "chatter",
                                                     chatterCallback,
                                                     rmw_qos_profile_default);
  // Subscribe to message using Topic string chatter
  rclcpp::spin(node);
  return 0;
}
