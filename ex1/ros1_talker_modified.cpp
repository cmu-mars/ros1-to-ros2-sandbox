// ROS 1 Talker in C++
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>


/** * This tutorial demonstrates simple sending of messages over the ROS system. */
int main(int argc, char **argv){
  ros::init(argc, argv, "talker_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Sleep for 2 seconds
  ros::Duration(1).sleep();
  
  ros::Rate loop_rate(10);
  int count = 1;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello World: " << count;
    msg.data = ss.str();
    std::cout << "Publishing: '" << msg.data << "'" << std::endl;
    // ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce(); //06/05/1710
    loop_rate.sleep();
    ++count;
    if (count == 11) {
      break;
    }
  }
  return 0;
}




