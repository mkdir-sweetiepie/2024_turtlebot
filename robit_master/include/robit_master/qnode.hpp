#ifndef ROBIT_MASTER_QNODE_HPP_
#define ROBIT_MASTER_QNODE_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <QThread>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <turtlebot3_msgs/msg/sensor_state.hpp>

#include "robit_driving.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
namespace robit_master {
extern bool button_clicked;
class QNode : public QThread {
  Q_OBJECT

 public:
  QNode();
  ~QNode();

  std_msgs::msg::Bool set_imu_is_on_;
  RobitDriving driving_;

  // int udp_socket_;
  // struct sockaddr_in server_addr_;
  // bool udp_initialized_;

  // void initUDP();
  // void sendUDP(const geometry_msgs::msg::Twist& cmd_vel);

 protected:
  void run();

 private:
  std::shared_ptr<rclcpp::Node> node_;

  void initPubSub();
  void visionCallback(const std::shared_ptr<robit_msgs::msg::VisionMsg> vision_msg);
  void buttonCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg_button);
  void imuCallback(const std_msgs::msg::UInt32::SharedPtr msg_imu);
  void cdsCallback(const turtlebot3_msgs::msg::SensorState::SharedPtr msg_cds);

  rclcpp::Publisher<robit_msgs::msg::MasterMsg>::SharedPtr pub_master_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_set_imu_;
 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor_;

  rclcpp::Subscription<robit_msgs::msg::VisionMsg>::SharedPtr sub_vision_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_button_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_imu_;
  rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr sub_cds_;

 Q_SIGNALS:
  void rosShutdown();
  void dataReceived();

 public Q_SLOTS:
  void turtleRun();
};

}  // namespace robit_master

#endif  // ROBIT_MASTER_QNODE_HPP_