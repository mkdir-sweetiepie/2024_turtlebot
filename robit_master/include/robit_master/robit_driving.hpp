#ifndef ROBIT_DRIVING_HPP
#define ROBIT_DRIVING_HPP

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "robit_msgs/msg/master_msg.hpp"
#include "robit_msgs/msg/simple_move_msg.hpp"
#include "robit_msgs/msg/vision_msg.hpp"

#define mode3  // mode1 : slow, mode2 : normal, mode3 : fast
#ifdef mode3
#define Linear_MAX 0.90
#define LINE_TURN_GAIN 0.045
#define STRAIGHT_GAIN 0.1
#define LEFT_CURVE_GAIN 0.047
#define RIGHT_CURVE_GAIN 0.05
#define P_GAIN 1.0  // 1.0
#define D_GAIN 6.0  // 6.0
#define STRAIGHT_LINEAR_INCRESE_GAIN 0.1
#define STRAIGHT_LINEAR_DECRESE_GAIN 0.08

#define LEFT_CURVE_GAIN2 0.2
#define RIGHT_CURVE_GAIN2 0.05
#define STRAIGHT_LINEAR_INCRESE_GAIN2 0.001
#define STRAIGHT_LINEAR_DECRESE_GAIN2 0.001
#define P_GAIN2 1.0  // 1.0
#define D_GAIN2 6.0  // 6.0

#define LEFT_CURVE_GAIN3 0.2
#define RIGHT_CURVE_GAIN3 0.2
#endif

#define ANGULAR_LINEAR_RATE3 2
#define ANGULAR_LINEAR_RATE2 4
#define ANGULAR_LINEAR_RATE 3
#define ANGLE_PIXEL_RATE 0.7
#define MAX_REVERSE 2.0

namespace robit_master {

class RobitDriving {
 public:
  RobitDriving();
  void go();
  void updateParameters(const std::shared_ptr<const robit_msgs::msg::VisionMsg>& vision_data, bool button_clicked);
  void setSpeed(double linear, double angular);

  robit_msgs::msg::MasterMsg master_msg_;
  robit_msgs::msg::VisionMsg Vision_msg_;
  static robit_msgs::msg::SimpleMoveMsg Simple_msg_;
  geometry_msgs::msg::Twist motor_value_;

  static bool start2024_;
  static int cds_data_;
  static int direction_angle_;

  static bool tunnel_starts;
  static bool navi_on_;
  static bool rviz_init;

  bool traffic_detect_ = false;
  bool cross_detect_ = false;
  int cross_info_;
  bool construct_detect_ = false;
  int construct_info_;
  bool parking_detect_ = false;
  int parking_info_;
  bool zigzag_detect_ = false;
  bool zigzag_info_;
  bool gatebar_detect_ = false;
  bool gatebar_info_;
  int just_before_tunnel_num_;

 private:
  void analyzeSituation();
  void lineTracing(double speed);
  void lineTracing2(double speed);
  void lineTracing3(double speed);
  void crossMotion();
  void constructMotion();
  void parkingMotion();

  bool button_click_;
  double linear_x_;
  double before_linear_x_;
  int left_line_x_, right_line_x_;
  bool right_line_detect_, left_line_detect_;
  double right_line_angle_, left_line_angle_;

  double rel_angle_;
  double rel_angle_ratio_;

  int situation_;
  int mission_sequence_[6];
  int bar_count_;

  int cnt = 0;

  enum Situation { NONE = 0, TRAFFIC, CROSS, CONSTRUCT, PARKING, ZIGZAG, GATEBAR, TUNNEL };
  enum CrossState { WAIT, TURN_LEFT, TURN_RIGHT, DRIVE, CROSS_END_LEFT, CROSS_END_RIGHT, CROSS_END };
  enum ConstructState { SLOW, LEFT__, LEFT_C1, FRONT_C1, FRONT_C2, FRONT_C3, CONSTRUCT_END };
  enum ParkingState { GO_P1, GO_P2, LEFT_P1, GO_P3, DETECT, TURN2L, TURN2R, BACK, TURN3L, TURN3R, GO_P4, LEFT_P2, PARKING_END };
};

}  // namespace robit_master

#endif  // ROBIT_DRIVING_HPP