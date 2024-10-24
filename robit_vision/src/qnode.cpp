#include "../include/robit_vision/qnode.hpp"

#include <cv_bridge/cv_bridge.h>

#include <QMetaType>
#include <numeric>
#include <stdexcept>

namespace robit_vision {

QNode::QNode() {
  rclcpp::init(0, nullptr);
  node = rclcpp::Node::make_shared("robit_vision");
  this->start();
  initPubSub();
  initUdpSocket();
  qRegisterMetaType<cv::Mat>("cv::Mat");

  connect(this, &QNode::ledReceived, this, &QNode::slotLed);
}
QNode::~QNode() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  if (udp_socket != -1) {
    close(udp_socket);
  }
}

void QNode::run() {
  rclcpp::WallRate loop_rate(60);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    receiveUdpImage();
    loop_rate.sleep();
  }
  Q_EMIT rosShutdown();
}

void QNode::initPubSub() {
  pubLed = node->create_publisher<std_msgs::msg::UInt8>("robit_led", 10);
  vision_pub = node->create_publisher<robit_msgs::msg::VisionMsg>("turtle_vision", 100);
  pubImage = node->create_publisher<sensor_msgs::msg::Image>("/camera/image", 1);

  subMaster = node->create_subscription<robit_msgs::msg::MasterMsg>("turtle_master", 100, std::bind(&QNode::updateParameter, this, std::placeholders::_1));
  subPsd = node->create_subscription<std_msgs::msg::UInt16MultiArray>("robit_psd", 10, std::bind(&QNode::psdCallback, this, std::placeholders::_1));
  subButton = node->create_subscription<std_msgs::msg::Int8MultiArray>("robit_button", 10, std::bind(&QNode::buttonCallback, this, std::placeholders::_1));
  subImu = node->create_subscription<std_msgs::msg::UInt32>("set_imu", 10, std::bind(&QNode::imuCallback, this, std::placeholders::_1));
  // subBBX = node->create_subscription<image_recognition_msgs::msg::BoundingBoxMsgs>("/camera/image/yolo_bboxes", 10, std::bind(&QNode::bboxCallback, this, std::placeholders::_1));
  // subYoloImage = node->create_subscription<sensor_msgs::msg::Image>("/camera/image/yolo_output", 10, std::bind(&QNode::yoloImageCallback, this, std::placeholders::_1));
}

void QNode::initUdpSocket() {
  udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket == -1) {
    throw std::runtime_error("Failed to create UDP socket");
  }

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(UDP_PORT);

  if (bind(udp_socket, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) == -1) {
    close(udp_socket);
    throw std::runtime_error("Failed to bind UDP socket");
  }

  RCLCPP_INFO(node->get_logger(), "UDP socket initialized and bound to port %d", UDP_PORT);
}

void QNode::receiveUdpImage() {
  std::vector<char> buffer(BUFFER_SIZE);
  sockaddr_in client_addr{};
  socklen_t client_addr_len = sizeof(client_addr);

  ssize_t received_bytes = recvfrom(udp_socket, buffer.data(), buffer.size(), 0, reinterpret_cast<struct sockaddr*>(&client_addr), &client_addr_len);

  if (received_bytes > 0) {
    std::vector<uchar> jpeg_data(buffer.begin(), buffer.begin() + received_bytes);
    cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

    if (!image.empty()) {
      processReceivedImage(image);
    }
  }
}

void QNode::processReceivedImage(const cv::Mat& image) {
  {
    std::lock_guard<std::mutex> lock(image_mutex);
    raw_img = image.clone();
  }

  // cross
  // if (robit_vision.cross_start) {  // cross start
  //   auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();
  //   pubImage->publish(*img_msg);
  // }
  if (robit_vision.cross_start) {
    robit_vision.Follow_bluesign(raw_img);
  }

  // if (Vision::cross_step[0]) {
  //   Vision::cross_condition = 0;  // cross stop
  // }
  if (Vision::cross_step[1]) {  // cross detect

    if (robit_vision.sign_condition[left]) {
      Vision::cross_condition = 1;
    } else if (robit_vision.sign_condition[right]) {
      Vision::cross_condition = 2;
    }
  }
  if (Vision::cross_step[2]) {
    Vision::cross_condition = 3;  // Drive
  }
  if (Vision::cross_step[3]) {  // cross turn
    if (robit_vision.sign_condition[left]) {
      Vision::cross_condition = 4;
    } else if (robit_vision.sign_condition[right]) {
      Vision::cross_condition = 5;
    }
  }
  if (Vision::cross_step[4]) {  // cross end
    Vision::cross_condition = 6;
  }

  // construct
  if (Vision::construct_start) {
    Vision::construct_condition = 0;  // left
  }
  if (Vision::construct_step[0]) {
    Vision::construct_condition = 1;  // left
  }
  if (Vision::construct_step[1]) {
    Vision::construct_condition = 2;  // left
  }
  if (Vision::construct_step[2]) {
    Vision::construct_condition = 3;  // front1
  }
  if (Vision::construct_step[3]) {
    Vision::construct_condition = 4;  // front2
  }
  if (Vision::construct_step[4]) {
    Vision::construct_condition = 5;  // front3
  }
  if (Vision::construct_step[5]) {
    Vision::construct_condition = 6;  // end
  }

  // parking
  if (Vision::parking_step[0]) {
    Vision::parking_condition = 1;  // go
  }
  if (Vision::parking_step[1]) {
    Vision::parking_condition = 2;  // left
  }
  if (Vision::parking_step[2]) {
    Vision::parking_condition = 3;  // go
  }
  if (Vision::parking_step[3]) {
    if (Vision::rotate) {
      Vision::parking_condition = 5;  // left
    } else {
      Vision::parking_condition = 6;  // right
    }
  }
  if (Vision::parking_step[4]) {
    Vision::parking_condition = 7;  // back
  }
  if (Vision::parking_step[5]) {
    if (Vision::rotate) {
      Vision::parking_condition = 8;  // left
    } else {
      Vision::parking_condition = 9;  // right
    }
  }

  if (Vision::parking_step[6]) {
    Vision::parking_condition = 10;  // go
  }
  if (Vision::parking_step[7]) {
    Vision::parking_condition = 11;  // turn
  }
  if (Vision::parking_step[8]) {
    Vision::parking_condition = 12;  // left
  }

  robit_vision.START(image);
  vision_pub->publish(robit_vision.Vision_msg);
  Q_EMIT imageReceived(image);
  Q_EMIT ledReceived();
  updateFps();
}
cv::Mat QNode::getLatestImage() const {
  std::lock_guard<std::mutex> lock(image_mutex);
  return raw_img.clone();
}

// void QNode::bboxCallback(const image_recognition_msgs::msg::BoundingBoxMsgs::SharedPtr msg) {
//   if (!robit_vision.direction_flag.load() && robit_vision.cross_start) {
//     if (msg->class_name == "left") {
//       robit_vision.direction_counter++;

//     } else if (msg->class_name == "right") {
//       robit_vision.direction_counter--;
//     }
//   }

//   if (std::abs(robit_vision.direction_counter) >= 100) {
//     CrossDirection new_direction = (robit_vision.direction_counter > 0) ? CrossDirection::LEFT : CrossDirection::RIGHT;
//     robit_vision.cross_direction.store(new_direction);
//     robit_vision.direction_flag.store(true);
//     std::cout << "Direction decided: " << (new_direction == CrossDirection::LEFT ? "LEFT" : "RIGHT") << std::endl;
//   }
// }
// void QNode::bboxCallback(const image_recognition_msgs::msg::BoundingBoxMsgs::SharedPtr msg) {
//   static int direction_change_counter = 0;
//   static int last_direction = 0;

//   if (!robit_vision.direction_flag.load() && robit_vision.cross_start) {
//     int current_direction = 0;
//     if (msg->class_name == "left") {
//       robit_vision.direction_counter++;
//       current_direction = 1;
//     } else if (msg->class_name == "right") {
//       robit_vision.direction_counter--;
//       current_direction = -1;
//     }

//     // 방향이 변경되었는지 확인
//     if (current_direction != 0 && current_direction != last_direction) {
//       direction_change_counter++;
//       last_direction = current_direction;
//     }
//   }

//   if (std::abs(robit_vision.direction_counter) >= 60 || direction_change_counter >= 30) {
//     CrossDirection new_direction;
//     if (direction_change_counter >= 30) {
//       new_direction = CrossDirection::RIGHT;  // 방향이 자주 변경되면 오른쪽으로 판단
//     } else {
//       new_direction = (robit_vision.direction_counter > 0) ? CrossDirection::LEFT : CrossDirection::RIGHT;
//     }
//     robit_vision.cross_direction.store(new_direction);
//     robit_vision.direction_flag.store(true);
//     std::cout << "Direction decided: " << (new_direction == CrossDirection::LEFT ? "LEFT" : "RIGHT") << std::endl;

//     // 결정 후 카운터 초기화
//     direction_change_counter = 0;
//     robit_vision.direction_counter = 0;
//   }
// }
// void QNode::yoloImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//   if (robit_vision.cross_start) {
//     try {
//       auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
//       Q_EMIT bluesignDetected(cv_ptr->image);
//     } catch (const cv_bridge::Exception& e) {
//       RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
//     }
//   }
// }

void QNode::updateParameter(const std::shared_ptr<const robit_msgs::msg::MasterMsg>& master_data) { robit_vision.update_parameter(master_data); }

void QNode::updateFps() {
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_update).count();

  if (duration > 0) {
    double current_fps = 1000.0 / duration;

    fps_history.push_back(current_fps);
    if (fps_history.size() > FPS_HISTORY_SIZE) {
      fps_history.pop_front();
    }

    double avg_fps = std::accumulate(fps_history.begin(), fps_history.end(), 0.0) / fps_history.size();

    Vision::now_fps = static_cast<int>(std::round(avg_fps));
    Q_EMIT fpsUpdated(Vision::now_fps);

    last_fps_update = now;
  }
}

void QNode::slotLed() {
  std_msgs::msg::UInt8 msg_led;
  int led_state = 0;
  led_state |= (Vision::led[0] & 0x01);
  led_state |= ((Vision::led[1] & 0x01) << 1);
  led_state |= ((Vision::led[2] & 0x01) << 2);
  led_state |= ((Vision::led[3] & 0x01) << 3);
  led_state |= ((Vision::led[4] & 0x01) << 4);
  led_state |= ((Vision::led[5] & 0x01) << 5);

  msg_led.data = led_state;
  pubLed->publish(msg_led);
}

void QNode::psdCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  if (msg->data.size() == PSD_SENSORS) {
    for (int i = 0; i < PSD_SENSORS; ++i) {
      Vision::psd[i] = movingAverageFilter(i, msg->data[i]);
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "Received incomplete PSD data");
  }
}

void QNode::buttonCallback(std_msgs::msg::Int8MultiArray::SharedPtr msg) {
  if (msg->data.size() == 8) {
    for (int i = 0; i < 0; ++i) {
      if (msg->data[i + 2] == 1) {
        // Vision::retry = i + 1;
        // robit_vision.Retry_Button();
        std::array<bool, 6> led_states{};
        led_states[i] = true;
        for (int j = 0; j < 6; ++j) {
          Vision::led[j] = led_states[j] ? 0 : 1;
        }
        break;
      }
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "Received incomplete BUTTON data");
  }
}

void QNode::imuCallback(const std_msgs::msg::UInt32::SharedPtr msg) {
  Vision::direction_angle = msg->data;

  if (Vision::rel_ctrl) {
    int temp_angle = (Vision::direction_angle >= Vision::rel_zero_angle) ? Vision::direction_angle - Vision::rel_zero_angle : 360 - (Vision::rel_zero_angle - Vision::direction_angle);
    Vision::rel_angle = temp_angle;
    Vision::rel_round = true;
  } else {
    Vision::rel_angle = Vision::direction_angle;
    Vision::rel_round = false;
  }
}

int QNode::movingAverageFilter(int num, int data) {
  static constexpr int WINDOW_SIZE = 5;
  static std::array<std::vector<int>, PSD_SENSORS> psd_buf;
  static std::array<bool, PSD_SENSORS> filter_Start{};
  static std::array<int, PSD_SENSORS> sum{};

  if (num < 0 || num >= PSD_SENSORS) {
    RCLCPP_ERROR(node->get_logger(), "Invalid sensor number in movingAverageFilter");
    return data;
  }

  if (psd_buf[num].size() < WINDOW_SIZE) {
    psd_buf[num].push_back(data);
    sum[num] += data;
    if (psd_buf[num].size() == WINDOW_SIZE) {
      filter_Start[num] = true;
    }
  } else {
    sum[num] -= psd_buf[num].front();
    sum[num] += data;
    psd_buf[num].erase(psd_buf[num].begin());
    psd_buf[num].push_back(data);
  }

  return filter_Start[num] ? sum[num] / WINDOW_SIZE : data;
}

}  // namespace robit_vision