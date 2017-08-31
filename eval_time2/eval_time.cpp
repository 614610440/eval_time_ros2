#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>



void chatterCallback(const std_msgs::msg::String::SharedPtr msg){
  std::cout << "I hear" << msg->data << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto talker = rclcpp::node::Node::make_shared("talker");
  auto listener = rclcpp::node::Node::make_shared("talker");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;


  auto topic = std::string("chatter"); //ここで送信受信する相手を決定

  rclcpp::WallRate loop_rate(5);//ループ中の待ち時間
  auto chatter_pub = talker->create_publisher<std_msgs::msg::String>(topic,custom_qos_profile);
  auto sub = listener->create_subscription<std_msgs::msg::String>(topic,chatterCallback,rmw_qos_profile_default);

  auto chatter_pub = talker->create_publisher<std_msgs::msg::String>(topic,custom_qos_profile);

  auto msg = std::make_shared<std_msgs::msg::String>();
  auto i=1;



  while(rclcpp::ok()){
  	msg->data = "Hello"+std::to_string(i++);
  	std::cout << "publish:" << msg->data <<std::endl;

  	chatter_pub->publish(msg);//msg
    loop_rate.sleep();
  	rclcpp::spin_some(listener);
  	loop_rate.sleep();

  }


  printf("hello world");
  return 0;
}
