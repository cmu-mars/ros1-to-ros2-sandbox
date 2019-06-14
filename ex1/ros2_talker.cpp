// ROS 2 Talker in C++                                                                                                                                                                                                                                                          
#include <iostream>
#include <memory>
// Need to change headers and naming convection                                                                                                                                                                                                                                 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv); // Initialization, followed by create object                                                                                                                                                                                                        
  auto node = rclcpp::Node::make_shared("talker");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", custom_qos_profile); // Create publisher with QoS profile                                                                                                                                         
  rclcpp::WallRate loop_rate(2);
  auto msg = std::make_shared<std_msgs::msg::String>(); // creating outgoing message                                                                                                                                                                                            
  auto i = 1;
  while (rclcpp::ok()) {
    msg->data = "Hello World: " + std::to_string(i++); // accessing the data in the pointer                                                                                                                                                                                     
    std::cout << "Publishing: '" << msg->data << "'" << std::endl;
    chatter_pub->publish(msg); // publish the message                                                                                                                                                                                                                           
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
