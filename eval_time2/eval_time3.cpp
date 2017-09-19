#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"
//#include "TimeScale.hpp"

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



class TimeScale {
  int count = 0;
public:
  std::shared_ptr<rclcpp::node::Node> talker;
  std::shared_ptr<rclcpp::node::Node> listener;
  std::string topic ;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr chatter_pub;
  sensor_msgs::msg::Image img;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

  rmw_qos_profile_t custom_qos_profile;
  //コンストラクタ
  TimeScale()
  {
  }

  void SendcountUp() {
    if (count < 100) {
      img.header.stamp = rclcpp::Time::now();//ヘッダに送信時間を登録
      //std::cout << "publish:" << img.width << std::endl;
      chatter_pub->publish(img);//msg
    } else {
      std::cout << "end chat" << std::endl;
    }
    count++;
  }
};


  
TimeScale Rosh;

//コールバック
void chatterCallbackImg(const sensor_msgs::msg::Image::SharedPtr msg) {
  //std::cout << "I hear :" << msg->height << std::endl;
  Rosh.SendcountUp();
}



int main(int argc, char * argv[])
{
  //初期化
  rclcpp::init(argc, argv);
  //TimeScale Rosh;

  //ノードの初期化
  Rosh.talker = rclcpp::node::Node::make_shared("talker");
  Rosh.listener = rclcpp::node::Node::make_shared("listener");
  Rosh.topic = std::string("chatter"); //ここで送信受信するトピックを決定

  // //QoSの設定
  Rosh.custom_qos_profile = rmw_qos_profile_reliable;  //今回使うQoS

  // //トピックを購読（監視）
  printf("Subscribing to topic '%s'\n", Rosh.topic.c_str());
  Rosh.sub          = Rosh.listener    ->create_subscription<sensor_msgs::msg::Image>(Rosh.topic, chatterCallbackImg, Rosh.custom_qos_profile);

  // //トピックへパブリッシュ（送信）
  printf("Publishing data on topic '%s'\n", Rosh.topic.c_str());
  Rosh.chatter_pub  = Rosh.talker      ->create_publisher<sensor_msgs::msg::Image>   (Rosh.topic, Rosh.custom_qos_profile);

  Rosh.img = sensor_msgs::msg::Image();
  Rosh.img.height = 150;
  Rosh.img.width  = 250;

  Rosh.SendcountUp();

  rclcpp::spin(Rosh.listener);
  return 0;
}


