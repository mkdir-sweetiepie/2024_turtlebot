#include "../include/set_imu/set_imu.hpp"

#include <cmath>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

SetImu::SetImu(rclcpp::Node::SharedPtr node) : node_(node) {
  set_imu_OnOff = node_->create_subscription<std_msgs::msg::Bool>("/set_imu_OnOff", 10, std::bind(&SetImu::OnOff, this, std::placeholders::_1));
  Imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>("/imu", 100, std::bind(&SetImu::imuMsgCallback, this, std::placeholders::_1));
  Imu_pub = node_->create_publisher<std_msgs::msg::UInt32>("/set_imu", 100);

  imu_calculated_once = false;
  set_imu_is_on = false;
  now_ads_angle = 0;
}

void SetImu::OnOff(const std_msgs::msg::Bool::SharedPtr msg) {
  if (imu_calculated_once) {
    bool before_OnOff = set_imu_is_on;
    set_imu_is_on = msg->data;

    if (!before_OnOff && set_imu_is_on) {
      zero_angle = now_ads_angle;
      RCLCPP_INFO(node_->get_logger(), "zero angle : %d", zero_angle);
    }
  }
}

void SetImu::imuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  double imu_x = msg->orientation.x;
  double imu_y = msg->orientation.y;
  double imu_z = msg->orientation.z;
  double imu_w = msg->orientation.w;

  double yaw = atan2(2 * imu_x * imu_y + 2 * imu_w * imu_z, imu_w * imu_w + imu_x * imu_x - imu_y * imu_y - imu_z * imu_z);
  double yaw_angle = yaw * RAD2DEG;

  if (yaw_angle > 0.0 && yaw_angle <= 180.0)
    now_ads_angle = yaw_angle;
  else if (yaw_angle <= 0.0 && yaw_angle > -180.0)
    now_ads_angle = yaw_angle + 360.0;

  if (set_imu_is_on) {
    int temp_angle;
    if (now_ads_angle >= zero_angle)
      temp_angle = now_ads_angle - zero_angle;
    else
      temp_angle = 360 - (zero_angle - now_ads_angle);
    set_now_angle.data = static_cast<uint32_t>(temp_angle);
  } else {
    set_now_angle.data = static_cast<uint32_t>(now_ads_angle);
  }
  Imu_pub->publish(set_now_angle);
  imu_calculated_once = true;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("set_imu_node");
  auto set_imu = std::make_shared<SetImu>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}