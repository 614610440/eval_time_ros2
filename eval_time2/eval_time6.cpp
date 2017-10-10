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
  static const int maxSendTimes=1000;

public:
  std::shared_ptr<rclcpp::node::Node> talker;
  std::shared_ptr<rclcpp::node::Node> listener;

  std::string topic ;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr chatter_pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;


  std_msgs::msg::Int32::SharedPtr msgPtr;
  std_msgs::msg::Int32 msg;

  rmw_qos_profile_t custom_qos_profile;

  bool usePtr=true;

  std::chrono::system_clock::time_point start;
  std::chrono::system_clock::time_point Ptrend;
  std::chrono::system_clock::time_point Datend;

  //コンストラクタ
  TimeScale()
  {
  }




  void startSendingData(int SendingStep){
    if (SendingStep==0){
      //msgPtr = std_msgs::msg::Int32::Ptr(new std_msgs::msg::Int32());
      msgPtr = std_msgs::msg::Int32::SharedPtr(new std_msgs::msg::Int32());
      count = 0;
      start = std::chrono::system_clock::now();
      usePtr = true;
      msgPtr->data = 10;
      SendcountUp();
    }else if (SendingStep==1){
      count = 0;
      Ptrend = std::chrono::system_clock::now();
      usePtr=false;
      msg.data = 20;
      SendcountUpData();
    }else{
      Datend =  std::chrono::system_clock::now();

      auto dur1 = Ptrend - start;        // 要した時間を計算
      auto dur2 = Datend - Ptrend;        // 要した時間を計算
      std::cout << maxSendTimes << " Data Sended" << std::endl;
      std::cout <<"pointer Send Time:" <<std::chrono::duration_cast<std::chrono::nanoseconds>(dur1).count() << " nano sec \n";
      std::cout <<"pointer Send Time:" <<std::chrono::duration_cast<std::chrono::nanoseconds>(dur2).count() << " nano sec \n";

    }
  }
  //custom_qos_profile

  void SendcountUp() {
    if (count < maxSendTimes) {//uniquepointe ポインタは終了時に自動的に破棄するから二回目以降エラーになってる？
      chatter_pub->publish(msgPtr);//msg
    }else{
      std::cout << "end chat Ptr" << std::endl;
      startSendingData(1);
    }
    count++;
  }

  void SendcountUpData(){
    if (count < maxSendTimes){
      chatter_pub->publish(msg);//msg
    }else{
      std::cout << "end chat Data" << std::endl;
      startSendingData(2);
    }
    count++;
  }

};

TimeScale Rosh;

//コールバック
void chatterCallbackImg(const std_msgs::msg::Int32::SharedPtr msgPtr) {
  //std::cout << "I hear :" << msgPtr->data << std::endl;

  //delete msgPtr;
  if (Rosh.usePtr==true){
    Rosh.SendcountUp();
  }else{
    Rosh.SendcountUpData();
  }


}



int main(int argc, char * argv[])
{
  //初期化
  rclcpp::init(argc, argv);

  //ノードの初期化
  Rosh.talker = rclcpp::node::Node::make_shared("talker");
  Rosh.listener = rclcpp::node::Node::make_shared("listener");
  Rosh.topic = std::string("chatter"); //ここで送信受信するトピックを決定

  // //QoSの設定
  Rosh.custom_qos_profile = rmw_qos_profile_best_effort;  //今回使うQoS

  // //トピックを購読（監視）
  printf("Subscribing to topic '%s'\n", Rosh.topic.c_str());
  Rosh.sub          = Rosh.listener    ->create_subscription<std_msgs::msg::Int32>(Rosh.topic, chatterCallbackImg, Rosh.custom_qos_profile);

  // //トピックへパブリッシュ（送信）
  printf("Publishing data on topic '%s'\n", Rosh.topic.c_str());
  Rosh.chatter_pub  = Rosh.talker      ->create_publisher<std_msgs::msg::Int32>   (Rosh.topic, Rosh.custom_qos_profile);

  Rosh.startSendingData(0);


  rclcpp::spin(Rosh.listener);
  return 0;
}


