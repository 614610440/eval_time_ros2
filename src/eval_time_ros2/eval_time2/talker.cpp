#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

static const rmw_qos_profile_t rmw_qos_profile_reliable = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,			//全サンプルを保存
  100,										//
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,		//送信を保証
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL	//サンプルの永続を保証
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("talker");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;

  auto topic = std::string("chatter"); //ここで送信受信する相手を決定

  auto chatter_pub = node->create_publisher<std_msgs::msg::String>(topic,custom_qos_profile);

  rclcpp::WallRate loop_rate(2);

  auto msg = std::make_shared<std_msgs::msg::String>();

  auto i=1;

  while(rclcpp::ok()){
  	msg->data = "Hello"+std::to_string(i++);
  	std::cout << "publish" << std::endl;

  	chatter_pub->publish(msg);//msg
  	rclcpp::spin_some(node);
  	loop_rate.sleep();

  }


  printf("hello world");
  return 0;
}
