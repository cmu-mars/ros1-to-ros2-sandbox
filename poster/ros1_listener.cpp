void chatterCallback(const std_msgs::String::ConstPtr& msg){
  // ...
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
