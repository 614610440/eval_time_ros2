#include <TimeScale.hpp>


  //コンストラクタ
  TimeScale::TimeScale(){
    rclcpp::init(argc, argv);
  }

  //コールバック
  void TimeScale::chatterCallbackImg(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::cout << "I hear :" << msg->height << std::endl;
  }


  void TimeScale::SendcountUp(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr chatter , sensor_msgs::msg::Image img) {
    if (count < 100) {
      img.header.stamp = rclcpp::Time::now();//ヘッダに送信時間を登録
      std::cout << "publish:" << img.width << std::endl;
      chatter->publish(img);//msg
    } else {
      std::cout << "end chat" << std::endl;
    }
    count++;
  }
