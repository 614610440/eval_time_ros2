

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"


std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void chatterCallbackImg(const sensor_msgs::msg::Image::SharedPtr msg){
  std::cout << "I hear :" << msg->height << std::endl;
}


int main(int argc, char * argv[])
{

  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default demo parameters
  bool show_camera = false;
  size_t depth = 10;
  double freq = 30.0;
  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  size_t width = 320;
  size_t height = 240;
  //std::string topic("image");



  // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
  //auto node = rclcpp::node::Node::make_shared("cam2image");
  auto talker = rclcpp::node::Node::make_shared("talker");
  auto listener = rclcpp::node::Node::make_shared("talker");
  auto topic = std::string("chatter"); //ここで送信受信する相手を決定
  rclcpp::WallRate loop_rate(5);//ループ中の待ち時間


  rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;
  custom_camera_qos_profile.depth = depth;
  custom_camera_qos_profile.reliability = reliability_policy;
  custom_camera_qos_profile.history = history_policy;


  printf("Subscribing to topic '%s'\n", topic.c_str());
  auto sub = listener->create_subscription<sensor_msgs::msg::Image>(topic, chatterCallbackImg, custom_camera_qos_profile);

  printf("Publishing data on topic '%s'\n", topic.c_str());
  auto chatter_pub = talker->create_publisher<sensor_msgs::msg::Image>(topic, custom_camera_qos_profile);

  sensor_msgs::msg::Image img = sensor_msgs::msg::Image();
  img.height = 100;
  img.width = 200;
  img.header.stamp = rclcpp::Time::now();
  

  size_t i = 1;

  while(rclcpp::ok()){
    img.header.stamp = rclcpp::Time::now();
   std::cout << "publish:" << img.width <<std::endl;
   chatter_pub->publish(img);//msg
    loop_rate.sleep();
   rclcpp::spin_some(listener);
   loop_rate.sleep();
  }



  return 0;
}
