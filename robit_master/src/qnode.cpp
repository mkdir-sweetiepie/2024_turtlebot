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
  move_sub_ = node_->create_subscription<robit_msgs::msg::SimpleMoveMsg>("simple", 10, std::bind(&QNode::SimpleDataCallback, this, std::placeholders::_1));
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
  if (driving_.rviz_init) rvizInit();
  if (is_tunnel) tunnelProcess();
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

void QNode::imuCallback(const std_msgs::msg::UInt32::SharedPtr msg_imu) { RobitDriving::direction_angle_ = msg_imu->data; }

void QNode::cdsCallback(const turtlebot3_msgs::msg::SensorState::SharedPtr msg_cds) {
  RobitDriving::cds_data_ = msg_cds->illumination;

  static int high_cds_count = 0;          // 1000 이상인 값을 카운트하는 변수
  static const int REQUIRED_COUNT = 10;   // 필요한 카운트 횟수
  static const int CDS_THRESHOLD = 1000;  // CDS 임계값

  // CDS 값이 1000 이상이면 카운트 증가
  if (RobitDriving::cds_data_ >= CDS_THRESHOLD) {
    high_cds_count++;
  } else {
    // CDS 값이 1000 미만이면 카운트 리셋
    high_cds_count = 0;
  }
  // // 터널 미션 체크
  // if (!RobitDriving::tunnel_starts && high_cds_count >= REQUIRED_COUNT && driving_.master_msg_.gatebar_done) {
  //   std::cout << "cds says it's tunnel now!" << std::endl;
  //   tunnelState = TUNNEL_INIT;
  //   is_tunnel = true;
  //   std::cout << "!!!!now rviz on!!!! : " << std::endl;
  //   RobitDriving::rviz_init = true;
  //   RobitDriving::navi_on_ = true;
  // }
}
// rviz를 켜는 함수
void QNode::rvizInit() {
  driving_.rviz_init = false;
  system("ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/last.yaml");
  // roslaunch turtlebot3_navigation turtlebot3_navigation.launch &
  msleep(300);
  // std::cout << "R-biz ON" << std::endl;
}

// simple_move에서 판별한 골 도달 상태를 받아오는 함수
void QNode::SimpleDataCallback(const robit_msgs::msg::SimpleMoveMsg::SharedPtr msg) {
  RobitDriving::Simple_msg_.mode = msg->mode;

  // 터널 내부이고, move_base에선 골에 도달하지 않았다고 판별할 때
  if (is_tunnel && !arrive_goal) {
    // simple_move에서 골에 도달했다고 판별 시 내비게이션 노드를 끔.
    if (RobitDriving::Simple_msg_.mode == 2) {
      std::cout << "simple_move_arrival!" << std::endl;
      // system("rosnode kill move_simple");
      // system("rosnode kill move_base_node");
      // system("rosnode kill move_base");
      arrive_goal = true;
    }
  }
}

void QNode::tunnelInit() {
  msleep(100);
  driving_.tunnel_starts = true;
}
void QNode::tunnelProcess() {}

}  // namespace robit_master