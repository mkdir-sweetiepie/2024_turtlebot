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
#include "robit_msgs/msg/simple_move_msg.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"

// 상수 정의
#define RAD2DEG (180.0 / M_PI)
#define THESH_RIGHT_DISTANCE 0.15  // 0.15
#define THESH_FRONT_DISTANCE 0.22  // 0.15
#define TURN_GAIN 0.025

namespace robit_master {
extern bool button_clicked;
class QNode : public QThread {
  Q_OBJECT

 public:
  QNode();
  ~QNode();

  std_msgs::msg::Bool set_imu_is_on_;
  RobitDriving driving_;

  bool is_tunnel, arrive_goal;
  int tunnelState, tunnelEscapeState;
  enum { TUNNEL_INIT, TUNNEL_MID, TUNNEL_END };
  enum { TUNNEL_WALL, TUNNEL_GATE, TUNNEL_ESCAPE };

  void rvizInit();
  void tunnelInit();
  void tunnelProcess();

  // // tunnel data
  // geometry_msgs::PoseWithCovarianceStamped initial_pose;
  // geometry_msgs::PoseStamped goal_pose;
  // actionlib_msgs::GoalID move_base_cancel;

  // int udp_socket_;
  // struct sockaddr_in server_addr_;
  // bool udp_initialized_;

  // void initUDP();
  // void sendUDP(const geometry_msgs::msg::Twist& cmd_vel);

 protected:
  void run();

 private:
  std::shared_ptr<rclcpp::Node> node_;

  // rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client;

  void initPubSub();
  void visionCallback(const std::shared_ptr<robit_msgs::msg::VisionMsg> vision_msg);
  void buttonCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg_button);
  void imuCallback(const std_msgs::msg::UInt32::SharedPtr msg_imu);
  void cdsCallback(const turtlebot3_msgs::msg::SensorState::SharedPtr msg_cds);
  void SimpleDataCallback(const robit_msgs::msg::SimpleMoveMsg::SharedPtr msg);

  rclcpp::Publisher<robit_msgs::msg::MasterMsg>::SharedPtr pub_master_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_set_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor_;

  rclcpp::Subscription<robit_msgs::msg::VisionMsg>::SharedPtr sub_vision_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_button_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_imu_;
  rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr sub_cds_;
  rclcpp::Subscription<robit_msgs::msg::SimpleMoveMsg>::SharedPtr move_sub_;

 Q_SIGNALS:
  void rosShutdown();
  void dataReceived();

 public Q_SLOTS:
  void turtleRun();
};

}  // namespace robit_master

#endif  // ROBIT_MASTER_QNODE_HPP_