#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>


void chatterCallback(const std_msgs::msg::String::SharedPtr msg){
  std::cout << "I hear" << msg->data << std::endl;
}



int main(int argc, char * argv[])
{


  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("listener");


  auto topic = std::string("chatter");

  auto sub = node->create_subscription<std_msgs::msg::String>(topic,chatterCallback,rmw_qos_profile_default);

  rclcpp::spin(node);

  return 0;
}
