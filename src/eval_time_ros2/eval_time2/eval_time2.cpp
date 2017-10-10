#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"


#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"


//QOSの設定（信頼）
static const rmw_qos_profile_t rmw_qos_profile_reliable = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,      //全サンプルを保存
  100,                                  //意味なし
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,    //送信を保証
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL //サンプルの永続を保証
};
//QOSの設定（ベストエフォート）
static const rmw_qos_profile_t rmw_qos_profile_best_effort = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,     //1こだけサンプルを保存
  1,                    //
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, //サンプルは消失する可能性
  RMW_QOS_POLICY_DURABILITY_VOLATILE    //永続は保証しない
};

static const rmw_qos_profile_t rmw_qos_profile_history = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,   //100こだけサンプルを保存
  100,                  // depth option for HISTORY
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,  //送信を保証
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL //サンプルの永続を保証
};

//コールバック
void chatterCallbackImg(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::cout << "I hear :" << msg->height << std::endl;
}



int main(int argc, char * argv[])
{

  //初期化
  rclcpp::init(argc, argv);

  //ノードの初期化
  auto talker = rclcpp::node::Node::make_shared("talker");
  auto listener = rclcpp::node::Node::make_shared("listener");
  auto topic = std::string("chatter"); //ここで送信受信するトピックを決定
  rclcpp::WallRate loop_rate(5);//ループ中の待ち時間


  //QoSの設定
  rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_reliable;  //今回使うQoS


  //トピックを購読（監視）
  printf("Subscribing to topic '%s'\n", topic.c_str());
  auto sub          = listener    ->create_subscription<sensor_msgs::msg::Image>(topic, chatterCallbackImg, custom_camera_qos_profile);

  //トピックへパブリッシュ（送信）
  printf("Publishing data on topic '%s'\n", topic.c_str());
  auto chatter_pub  = talker      ->create_publisher<sensor_msgs::msg::Image>   (topic, custom_camera_qos_profile);

  sensor_msgs::msg::Image img = sensor_msgs::msg::Image();
  img.height = 150;
  img.width = 250;


  //size_t i = 1;
  
  //rclcpp::spin(listener);
  while (rclcpp::ok()) {
    img.header.stamp = rclcpp::Time::now();//ヘッダに送信時間を登録
    std::cout << "publish:" << img.width << std::endl;
    chatter_pub->publish(img);//msg
    loop_rate.sleep();
    rclcpp::spin_some(listener);
    loop_rate.sleep();
  }



  return 0;
}
