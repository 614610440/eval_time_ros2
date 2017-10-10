#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

//https://github.com/ros2/ros2/wiki/Intra-Process-Communicationa
//

void chatterCallbackPtr(const std_msgs::msg::Int32::SharedPtr msgPtr);

void chatterCallbackDat(const sensor_msgs::msg::Image::SharedPtr msg);

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
  static const int maxSendTimes = 10;

public:
  std::shared_ptr<rclcpp::node::Node> talker;
  std::shared_ptr<rclcpp::node::Node> listener;

  std::string topic ;
  std::string topic2 ;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr chatter_pub_ptr;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_ptr;

  // rclcpp::Publisher<std_msgs::msg::Int32>::Int32 chatter_dat;
  // rclcpp::Subscription<std_msgs::msg::Int32>::Int32 sub_dat;

  std_msgs::msg::Int32::SharedPtr msgPtr;
  std_msgs::msg::Int32 msg;

  rmw_qos_profile_t custom_qos_profile;

  bool usePtr = true;

  std::chrono::system_clock::time_point start;
  std::chrono::system_clock::time_point Ptrend;
  std::chrono::system_clock::time_point Datend;

  //コンストラクタ
  TimeScale()
  {
  }


  void setTopic() {

    // //トピックを購読（監視）
    printf("Subscribing to topic '%s'\n", topic.c_str());
    //sub_ptr          = listener    ->create_subscription<std_msgs::msg::Int32>(topic, chatterCallbackPtr, custom_qos_profile);
    // //トピックへパブリッシュ（送信）
    printf("Publishing data on topic '%s'\n", topic.c_str());
    //chatter_pub_ptr  = talker      ->create_publisher<std_msgs::msg::Int32>   (topic, custom_qos_profile);
    // //トピックを購読（監視）
    //printf("Subscribing to topic '%s'\n", topic.c_str());
    auto sub_dat          = listener    ->create_subscription<sensor_msgs::msg::Image>(topic2, chatterCallbackDat, custom_qos_profile);
    // //トピックへパブリッシュ（送信）
    //printf("Publishing data on topic '%s'\n", topic.c_str());
    auto chatter_dat  = talker      ->create_publisher<sensor_msgs::msg::Image> (topic2, custom_qos_profile);

  }


  void startSendingData(int SendingStep) {
    if (SendingStep == 0) {


      //msgPtr = std_msgs::msg::Int32::SharedPtr(new std_msgs::msg::Int32());
      count = 0;
      start = std::chrono::system_clock::now();
      usePtr = true;
      SendcountUp();
    } else if (SendingStep == 1) {





      count = 0;
      Ptrend = std::chrono::system_clock::now();
      usePtr = false;
      SendcountUpData();
    } else {
      Datend =  std::chrono::system_clock::now();

      auto dur1 = Ptrend - start;        // 要した時間を計算
      auto dur2 = Datend - Ptrend;        // 要した時間を計算
      std::cout << maxSendTimes << " Data Sended" << std::endl;
      std::cout << "pointer Send Time:" << std::chrono::duration_cast<std::chrono::nanoseconds>(dur1).count() << " nano sec \n";
      std::cout << "Data    Send Time:" << std::chrono::duration_cast<std::chrono::nanoseconds>(dur2).count() << " nano sec \n";


    }
  }

  void SendcountUp() {
    // if (count < maxSendTimes) {//uniquepointe ポインタは終了時に自動的に破棄するか
    //   //msg.header.stamp = rclcpp::Time::now();//ヘッダに送信時間を登録

    //   msgPtr->data = count;
    //   //std::cout << "publish:" << msgPtr->data << std::endl;
    //   chatter_pub_ptr->publish(msgPtr);//msg
    // } else {
    //   std::cout << "end chat Ptr" << std::endl;
    //   startSendingData(1);
    // }
    // count++;
  }

  void SendcountUpData() {
    // if (count < maxSendTimes) {
    //   msg.data = count;
    //   //std::cout << "publish:Data" << msg.data << std::endl;
    //   chatter_pub_dat->publish(msg);//msg
    // } else {
    //   std::cout << "end chat Data" << std::endl;
    //   startSendingData(2);
    // }
    // count++;
  }

};

TimeScale Rosh;

//コールバック
void chatterCallbackPtr(const std_msgs::msg::Int32::SharedPtr msgPtr) {
  //std::cout << "I hear :" << msgPtr->data << std::endl;
  printf(
    "Published message with value: %d, and address: %0x", msgPtr->data,
    reinterpret_cast<std::uintptr_t>(msgPtr.get()));
  std::cout << std::endl;

  Rosh.SendcountUp();
}



void chatterCallbackDat(const sensor_msgs::msg::Image::SharedPtr msg) {
  // printf(
  //   "Published message with value: %d, and address: %0x", msg.data,
  //   reinterpret_cast<std::uintptr_t>(msg.get()));
  std::cout << std::endl;
  Rosh.SendcountUpData();
}


int main(int argc, char * argv[])
{
  //初期化
  rclcpp::init(argc, argv);

  //ノードの初期化
  Rosh.talker = rclcpp::node::Node::make_shared("talker");
  Rosh.listener = rclcpp::node::Node::make_shared("listener");
  Rosh.topic = std::string("chatter"); //ここで送信受信するトピックを決定
  Rosh.topic2 = std::string("chatter2"); //ここで送信受信するトピックを決定

  // //QoSの設定
  Rosh.custom_qos_profile = rmw_qos_profile_reliable;  //今回使うQoS



  Rosh.setTopic();

  Rosh.startSendingData(0);

  rclcpp::spin(Rosh.listener);
  return 0;
}


