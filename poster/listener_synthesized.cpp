void chatterCallback(const std_msgs::msg::String::SharedPtr msg){
  // ...
}

int main(int argc, char **argv){
  std::string node_name = "listener_node";
  rclcpp::init(argc, argv);
  auto the_freshest_id_0 = rclcpp::Node::make_shared(node_name);
  std::string topic_name = "chatter";
  int queue_size = 1000;
  auto the_freshest_id_1 = the_freshest_id_0->create_subscription<std_msgs::msg::String>(topic_name, chatterCallback, queue_size);
  rclcpp::spin(the_freshest_id_0);
  return 0;
}

