#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

class TimeScale {

  int count = 0;
public:
  // rclcpp::Node talker;
  // rclcpp::Node listener;
  // std::string  topic = "chatter";
  // rmw_qos_profile_t custom_qos_profile;
  //コンストラクタ
  TimeScale();


  void chatterCallbackImg(const sensor_msgs::msg::Image::SharedPtr msg);

  void SendcountUp(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr chatter , sensor_msgs::msg::Image img);


};