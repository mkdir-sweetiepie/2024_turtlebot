#ifndef SET_IMU_HPP
#define SET_IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <memory>

class SetImu
{
public:
  SetImu(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_imu_OnOff;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr Imu_sub;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr Imu_pub;

  bool imu_calculated_once;
  int now_ads_angle;
  bool set_imu_is_on;
  int zero_angle;
  std_msgs::msg::UInt32 set_now_angle;

  void OnOff(const std_msgs::msg::Bool::SharedPtr msg);
  void imuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif  // SET_IMU_HPP