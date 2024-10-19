#include "../include/robit_master/qnode.hpp"

namespace robit_master {
bool button_clicked = false;
QNode::QNode() {
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node_ = rclcpp::Node::make_shared("robit_master");
  this->start();
  initPubSub();
  // initUDP();
  QObject::connect(this, &QNode::dataReceived, this, &QNode::turtleRun);
}

QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  // if (udp_initialized_) {
  //   close(udp_socket_);
  // }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutdown();
}

void QNode::initPubSub() {
  pub_master_ = node_->create_publisher<robit_msgs::msg::MasterMsg>("turtle_master", 100);
  pub_set_imu_ = node_->create_publisher<std_msgs::msg::Bool>("set_imu_OnOff", 10);
  pub_motor_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  sub_vision_ = node_->create_subscription<robit_msgs::msg::VisionMsg>("turtle_vision", 100, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  sub_button_ = node_->create_subscription<std_msgs::msg::Int8MultiArray>("robit_button", 10, std::bind(&QNode::buttonCallback, this, std::placeholders::_1));
  sub_imu_ = node_->create_subscription<std_msgs::msg::UInt32>("set_imu", 10, std::bind(&QNode::imuCallback, this, std::placeholders::_1));
  sub_cds_ = node_->create_subscription<turtlebot3_msgs::msg::SensorState>("sensor_state", 10, std::bind(&QNode::cdsCallback, this, std::placeholders::_1));

  set_imu_is_on_.data = false;
}

// void QNode::initUDP() {
//   udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
//   if (udp_socket_ < 0) {
//     RCLCPP_ERROR(node_->get_logger(), "UDP 소켓 생성 실패");
//     return;
//   }

//   memset(&server_addr_, 0, sizeof(server_addr_));
//   server_addr_.sin_family = AF_INET;
//   server_addr_.sin_port = htons(7410);                        // 예제와 동일한 포트 사용
//   server_addr_.sin_addr.s_addr = inet_addr("192.168.168.6");  // 예제와 동일한 IP 사용

//   udp_initialized_ = true;
// }

// void QNode::sendUDP(const geometry_msgs::msg::Twist& cmd_vel) {
//   if (!udp_initialized_) {
//     return;
//   }

//   // cmd_vel에서 linear.x와 angular.z만 전송
//   char buffer[2 * sizeof(double)];
//   int offset = 0;

//   memcpy(buffer + offset, &cmd_vel.linear.x, sizeof(double));
//   offset += sizeof(double);

//   memcpy(buffer + offset, &cmd_vel.angular.z, sizeof(double));
//   offset += sizeof(double);

//   sendto(udp_socket_, buffer, offset, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
// }

void QNode::visionCallback(const std::shared_ptr<robit_msgs::msg::VisionMsg> vision_msg) {
  if (vision_msg) {
    driving_.updateParameters(vision_msg, button_clicked);
    Q_EMIT dataReceived();
  }
}

void QNode::turtleRun() {
  driving_.go();
   pub_motor_->publish(driving_.motor_value_);
  // sendUDP(driving_.motor_value_);
  pub_master_->publish(driving_.master_msg_);
}

void QNode::buttonCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg_button) {
  if (msg_button->data.size() == 8) {
    if (msg_button->data[0] == 1) {
      RobitDriving::start2024_ = true;
      RCLCPP_INFO(node_->get_logger(), "on : %d", RobitDriving::start2024_);
    }
    if (msg_button->data[1] == 1) {
      RobitDriving::start2024_ = false;
      RCLCPP_INFO(node_->get_logger(), "on : %d", RobitDriving::start2024_);
      set_imu_is_on_.data = false;
      pub_set_imu_->publish(set_imu_is_on_);
      set_imu_is_on_.data = true;
      pub_set_imu_->publish(set_imu_is_on_);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Received incomplete BUTTON data");
  }
}

void QNode::imuCallback(const std_msgs::msg::UInt32::SharedPtr msg_imu) {
  RobitDriving::direction_angle_ = msg_imu->data;
  if (driving_.turn_on_) {
    driving_.turn_on_ = driving_.imuTurn();
  }
}

void QNode::cdsCallback(const turtlebot3_msgs::msg::SensorState::SharedPtr msg_cds) { RobitDriving::cds_data_ = msg_cds->illumination; }

}  // namespace robit_master