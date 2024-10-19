#ifndef ROBIT_VISION_QNODE_HPP_
#define ROBIT_VISION_QNODE_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <QThread>
#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "image_recognition_msgs/msg/bounding_box_msgs.hpp"
#include "robit_msgs/msg/master_msg.hpp"
#include "robit_msgs/msg/vision_msg.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "vision.hpp"

namespace robit_vision {

class QNode : public QThread {
  Q_OBJECT

 public:
  QNode();
  virtual ~QNode();

  Vision robit_vision;
  cv::Mat getLatestImage() const;
  void updateParameter(const std::shared_ptr<const robit_msgs::msg::MasterMsg>& master_data);

  static constexpr int UDP_PORT = 3512;
  static constexpr int BUFFER_SIZE = 65507;
  static constexpr int FPS_HISTORY_SIZE = 10;
  static constexpr int PSD_SENSORS = 3;

  std::string packagePath = "/home/hyeon/ros2_ws/src/2024_turtlebot/robit_vision/robotis"; //robotis

 protected:
  void run() override;

 private:
  std::shared_ptr<rclcpp::Node> node;
  int udp_socket{-1};
  cv::Mat raw_img;
  mutable std::mutex image_mutex;

  std::chrono::steady_clock::time_point last_fps_update;
  std::deque<double> fps_history;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pubLed;
  rclcpp::Publisher<robit_msgs::msg::VisionMsg>::SharedPtr vision_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImage;

  rclcpp::Subscription<robit_msgs::msg::MasterMsg>::SharedPtr subMaster;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subPsd;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subButton;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subImu;
  rclcpp::Subscription<image_recognition_msgs::msg::BoundingBoxMsgs>::SharedPtr subBBX;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subYoloImage;

  void initPubSub();
  void initUdpSocket();
  void receiveUdpImage();
  void updateFps();

  int movingAverageFilter(int num, int data);

  void processReceivedImage(const cv::Mat& image);
  void psdCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void buttonCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
  void imuCallback(const std_msgs::msg::UInt32::SharedPtr msg);
  void bboxCallback(const image_recognition_msgs::msg::BoundingBoxMsgs::SharedPtr msg);
  void yoloImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

 Q_SIGNALS:
  void rosShutdown();
  void imageReceived(const cv::Mat& image);
  void ledReceived();
  void fpsUpdated(int fps);
  void bluesignDetected(const cv::Mat& image);

 public Q_SLOTS:
  void slotLed();
};

}  // namespace robit_vision

#endif  // ROBIT_VISION_QNODE_HPP_