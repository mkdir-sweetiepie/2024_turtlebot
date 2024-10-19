#include "../include/robit_master/robit_driving.hpp"

#include <cmath>

namespace robit_master {

int MIDDLE_OF_IMAGE = 158;

bool RobitDriving::start2024_ = false;
int RobitDriving::cds_data_ = 0;
int RobitDriving::direction_angle_ = 0;

RobitDriving::RobitDriving() : button_click_(false), bar_count_(0), situation_(NONE), zigzag_starts_(false), tunnel_starts_(false), navi_on_(false), now_mission_(0) {
  master_msg_.traffic_done = false;
  master_msg_.cross_done = false;
  master_msg_.gatebar_done = false;
  master_msg_.parking_done = false;
  master_msg_.construct_done = false;
  master_msg_.zigzag_done = false;
  master_msg_.tunnel_done = false;
  master_msg_.led_init = false;
}

void RobitDriving::setSpeed(double linear, double angular) {
  motor_value_.linear.x = linear;
  motor_value_.angular.z = angular;
}

void RobitDriving::go() {
  if (!start2024_) {
    setSpeed(0.0, 0.0);
    return;
  }
  if (start2024_ && traffic_detect_) {
    master_msg_.traffic_done = true;
    analyzeSituation();

    switch (situation_) {
      case NONE:
        lineTracing(0.24);

        break;
      case CROSS:
        crossMotion();
        break;
      case CONSTRUCT:
        constructMotion();
        break;
      case PARKING:
        parkingMotion();
        break;
      case ZIGZAG:
        zigzag_starts_ = true;
        break;
      case GATEBAR:
        bar_count_++;
        break;
      default:
        break;
    }
  }
}

void RobitDriving::analyzeSituation() {
  if (cross_detect_ && !master_msg_.cross_done) {
    situation_ = CROSS;
  } else if (construct_detect_ && !master_msg_.construct_done) {
    situation_ = CONSTRUCT;
  } else if (parking_detect_ && !master_msg_.parking_done) {
    situation_ = PARKING;
  } else if (zigzag_detect_ && !master_msg_.zigzag_done) {
    situation_ = ZIGZAG;
  } else if (gatebar_detect_ && !master_msg_.gatebar_done) {
    situation_ = GATEBAR;
  } else {
    situation_ = NONE;
  }
}

void RobitDriving::retry() {
  if (Vision_msg_.retry > 0) {
    // 크로스 미션 재설정
    if (Vision_msg_.retry <= mission_sequence_[CROSS]) {
      master_msg_.cross_done = false;
      cross_condition_ = 0;
    }

    // 건설 구간 미션 재설정
    if (Vision_msg_.retry <= mission_sequence_[CONSTRUCT]) {
      master_msg_.construct_done = false;
      construct_condition_ = 0;
    }

    // 주차 미션 재설정
    if (Vision_msg_.retry <= mission_sequence_[PARKING]) {
      master_msg_.parking_done = false;
      parking_condition_ = 0;
    }

    // 지그재그 미션 재설정
    if (Vision_msg_.retry <= mission_sequence_[ZIGZAG]) {
      master_msg_.zigzag_done = false;
      zigzag_starts_ = false;
    }

    // 차단바 미션 재설정
    if (Vision_msg_.retry <= mission_sequence_[GATEBAR]) {
      master_msg_.gatebar_done = false;
      bar_count_ = 0;
    }

    // 터널 미션 재설정
    if (Vision_msg_.retry <= mission_sequence_[TUNNEL]) {
      master_msg_.tunnel_done = false;
      tunnel_starts_ = false;
    }

    // 공통 상태 재설정
    start2024_ = false;
    situation_ = NONE;

    // 현재 미션 번호 업데이트
    now_mission_ = Vision_msg_.retry;

    // 모터 정지
    setSpeed(0.0, 0.0);

    std::cout << "Retry initiated. Current mission: " << now_mission_ << std::endl;
  }
}

void RobitDriving::updateParameters(const std::shared_ptr<const robit_msgs::msg::VisionMsg>& Vision_msg_vision_data, bool button_clicked) {
  Vision_msg_ = *Vision_msg_vision_data;

  // 라인 관련 정보 업데이트
  right_line_x_ = Vision_msg_.r_diff_pixel;
  left_line_x_ = Vision_msg_.l_diff_pixel;
  right_line_angle_ = Vision_msg_.r_angle;
  left_line_angle_ = Vision_msg_.l_angle;
  right_line_detect_ = Vision_msg_.r_line_info;
  left_line_detect_ = Vision_msg_.l_line_info;

  // 트래픽 신호 처리
  traffic_detect_ = Vision_msg_.traffic;

  // 미션 관련 정보 업데이트
  cross_detect_ = Vision_msg_.cross_detect;
  cross_info_ = Vision_msg_.cross_info;
  construct_detect_ = Vision_msg_.construct_detect;
  construct_info_ = Vision_msg_.construct_info;
  parking_detect_ = Vision_msg_.parking_detect;
  parking_info_ = Vision_msg_.parking_info;
  zigzag_detect_ = Vision_msg_.zigzag_detect;
  slow_zone_ = Vision_msg_.slow_zone;
  gatebar_detect_ = Vision_msg_.gatebar_detect;
  just_before_tunnel_num_ = Vision_msg_.just_before_tunnel_num;

  // IMU 관련 정보 업데이트
  rel_angle_ = Vision_msg_.rel_angle;
  rel_angle_ratio_ = Vision_msg_.rel_angle_ratio;

  // 미션 시퀀스 업데이트
  for (int i = 0; i < 6; i++) {
    mission_sequence_[i] = Vision_msg_.mission_sequence[i];
  }
  retry_ = Vision_msg_.retry;

  button_click_ = button_clicked;

  // 상황 분석 실행
  analyzeSituation();
}

void RobitDriving::lineTracing(double speed) {
  // std::cout << "lineTracing" << std::endl;
  double f1 = 0.3;
  double f2 = 1.0 - f1;

  linear_x_ = speed;
  static double pre_pixel_gap = 0.0;
  static int pixel_offset;
  static double angular_z = 0.0;
  double pixel_gap = 0.0;

  double min_angle = 0.1;
  double max_angle = 3.6;
  double min_linear_v = speed * 0.3;
  double max_linear_v = speed * 0.7;

  if (right_line_detect_ && left_line_detect_) {
    double middlePoint = (left_line_x_ + right_line_x_) / 2.0;
    double middleAngle = (right_line_angle_ + left_line_angle_) / 2.0;
    pixel_offset = (320 - right_line_x_ + left_line_x_) / 2.0;
    pixel_gap = ((middleAngle - 90.0) * (1.0 - 0.1) + ((double)160.0 - middlePoint) * 0.1) * STRAIGHT_GAIN;
  } else if (right_line_detect_ && !left_line_detect_) {
    pixel_gap = ((right_line_angle_ - 90.0) * (1.0 - ANGLE_PIXEL_RATE) + ((double)(320 - pixel_offset) - right_line_x_) * ANGLE_PIXEL_RATE) * LEFT_CURVE_GAIN;
    if (pixel_gap < 0) {
      if (pixel_gap < -MAX_REVERSE) {
        pixel_gap = -MAX_REVERSE;
      }
      pixel_gap = -pixel_gap;
      pre_pixel_gap = pixel_gap;

      angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;
      if (angular_z > max_angle) {
        angular_z = max_angle;
      } else if (angular_z < -max_angle) {
        angular_z = -max_angle;
      }
      setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE);
      return;
    }

  } else if (!right_line_detect_ && left_line_detect_) {
    pixel_gap = ((left_line_angle_ - 90.0) * (1.0 - ANGLE_PIXEL_RATE) + ((double)pixel_offset - left_line_x_) * ANGLE_PIXEL_RATE) * RIGHT_CURVE_GAIN;
    if (pixel_gap > 0) {
      if (pixel_gap > MAX_REVERSE) {
        pixel_gap = MAX_REVERSE;
      }
      pixel_gap = -pixel_gap;
      pre_pixel_gap = -pixel_gap;

      angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;
      if (angular_z > max_angle) {
        angular_z = max_angle;
      } else if (angular_z < -max_angle) {
        angular_z = -max_angle;
      }
      setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE);
      return;
    }

  } else {
    pre_pixel_gap = 0;
  }

  angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;

  if (angular_z > max_angle) {
    angular_z = max_angle;
  } else if (angular_z < -max_angle) {
    angular_z = -max_angle;
  }

  if (fabs(angular_z) > min_angle) {
    linear_x_ = (((max_linear_v - min_linear_v) / (min_angle - max_angle)) * (fabs(angular_z) - min_angle)) + max_linear_v;
  }

  if (before_linear_x_ > linear_x_ + STRAIGHT_LINEAR_DECRESE_GAIN) {
    before_linear_x_ -= STRAIGHT_LINEAR_DECRESE_GAIN;
  } else if (before_linear_x_ < linear_x_ - STRAIGHT_LINEAR_INCRESE_GAIN) {
    before_linear_x_ += STRAIGHT_LINEAR_INCRESE_GAIN;
  } else {
    before_linear_x_ = linear_x_;
  }

  setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE);

  pre_pixel_gap = pixel_gap;
}
void RobitDriving::lineTracing2(double speed) {
  // std::cout << "lineTracing" << std::endl;
  double f1 = 0.3;
  double f2 = 1.0 - f1;

  linear_x_ = speed;
  static double pre_pixel_gap = 0.0;
  static int pixel_offset;
  static double angular_z = 0.0;
  double pixel_gap = 0.0;

  double min_angle = 0.1;
  double max_angle = 3.6;
  double min_linear_v = speed * 0.3;
  double max_linear_v = speed * 0.7;

  if (right_line_detect_ && left_line_detect_) {
    double middlePoint = (left_line_x_ + right_line_x_) / 2.0;
    double middleAngle = (right_line_angle_ + left_line_angle_) / 2.0;
    pixel_offset = (320 - right_line_x_ + left_line_x_) / 2.0;
    pixel_gap = ((middleAngle - 90.0) * (1.0 - 0.1) + ((double)160.0 - middlePoint) * 0.1) * STRAIGHT_GAIN;
  } else if (right_line_detect_ && !left_line_detect_) {
    pixel_gap = ((right_line_angle_ - 90.0) * (1.0 - ANGLE_PIXEL_RATE) + ((double)(320 - pixel_offset) - right_line_x_) * ANGLE_PIXEL_RATE) * LEFT_CURVE_GAIN2;
    if (pixel_gap < 0) {
      if (pixel_gap < -MAX_REVERSE) {
        pixel_gap = -MAX_REVERSE;
      }
      pixel_gap = -pixel_gap;
      pre_pixel_gap = pixel_gap;

      angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;
      if (angular_z > max_angle) {
        angular_z = max_angle;
      } else if (angular_z < -max_angle) {
        angular_z = -max_angle;
      }
      setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE2);
      return;
    }

  } else if (!right_line_detect_ && left_line_detect_) {
    pixel_gap = ((left_line_angle_ - 90.0) * (1.0 - ANGLE_PIXEL_RATE) + ((double)pixel_offset - left_line_x_) * ANGLE_PIXEL_RATE) * RIGHT_CURVE_GAIN2;
    if (pixel_gap > 0) {
      if (pixel_gap > MAX_REVERSE) {
        pixel_gap = MAX_REVERSE;
      }
      pixel_gap = -pixel_gap;
      pre_pixel_gap = -pixel_gap;

      angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;
      if (angular_z > max_angle) {
        angular_z = max_angle;
      } else if (angular_z < -max_angle) {
        angular_z = -max_angle;
      }
      setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE2);
      return;
    }

  } else {
    pre_pixel_gap = 0;
  }

  angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;

  if (angular_z > max_angle) {
    angular_z = max_angle;
  } else if (angular_z < -max_angle) {
    angular_z = -max_angle;
  }

  if (fabs(angular_z) > min_angle) {
    linear_x_ = (((max_linear_v - min_linear_v) / (min_angle - max_angle)) * (fabs(angular_z) - min_angle)) + max_linear_v;
  }

  if (before_linear_x_ > linear_x_ + STRAIGHT_LINEAR_DECRESE_GAIN2) {
    before_linear_x_ -= STRAIGHT_LINEAR_DECRESE_GAIN2;
  } else if (before_linear_x_ < linear_x_ - STRAIGHT_LINEAR_INCRESE_GAIN2) {
    before_linear_x_ += STRAIGHT_LINEAR_INCRESE_GAIN2;
  } else {
    before_linear_x_ = linear_x_;
  }

  setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE2);

  pre_pixel_gap = pixel_gap;
}
void RobitDriving::lineTracing3(double speed) {
  // std::cout << "lineTracing" << std::endl;
  double f1 = 0.3;
  double f2 = 1.0 - f1;

  linear_x_ = speed;
  static double pre_pixel_gap = 0.0;
  static int pixel_offset;
  static double angular_z = 0.0;
  double pixel_gap = 0.0;

  double min_angle = 0.1;
  double max_angle = 3.6;
  double min_linear_v = speed * 0.3;
  double max_linear_v = speed * 0.7;

  if (right_line_detect_ && left_line_detect_) {
    double middlePoint = (left_line_x_ + right_line_x_) / 2.0;
    double middleAngle = (right_line_angle_ + left_line_angle_) / 2.0;
    pixel_offset = (320 - right_line_x_ + left_line_x_) / 2.0;
    pixel_gap = ((middleAngle - 90.0) * (1.0 - 0.1) + ((double)160.0 - middlePoint) * 0.1) * STRAIGHT_GAIN;
  } else if (right_line_detect_ && !left_line_detect_) {
    pixel_gap = ((right_line_angle_ - 90.0) * (1.0 - ANGLE_PIXEL_RATE) + ((double)(320 - pixel_offset) - right_line_x_) * ANGLE_PIXEL_RATE) * LEFT_CURVE_GAIN3;
    if (pixel_gap < 0) {
      if (pixel_gap < -MAX_REVERSE) {
        pixel_gap = -MAX_REVERSE;
      }
      pixel_gap = -pixel_gap;
      pre_pixel_gap = pixel_gap;

      angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;
      if (angular_z > max_angle) {
        angular_z = max_angle;
      } else if (angular_z < -max_angle) {
        angular_z = -max_angle;
      }
      setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE3);
      return;
    }

  } else if (!right_line_detect_ && left_line_detect_) {
    pixel_gap = ((left_line_angle_ - 90.0) * (1.0 - ANGLE_PIXEL_RATE) + ((double)pixel_offset - left_line_x_) * ANGLE_PIXEL_RATE) * RIGHT_CURVE_GAIN3;
    if (pixel_gap > 0) {
      if (pixel_gap > MAX_REVERSE) {
        pixel_gap = MAX_REVERSE;
      }
      pixel_gap = -pixel_gap;
      pre_pixel_gap = -pixel_gap;

      angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;
      if (angular_z > max_angle) {
        angular_z = max_angle;
      } else if (angular_z < -max_angle) {
        angular_z = -max_angle;
      }
      setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE3);
      return;
    }

  } else {
    pre_pixel_gap = 0;
  }

  angular_z = f2 * (pixel_gap * P_GAIN + (pixel_gap - pre_pixel_gap) * D_GAIN) + f1 * angular_z;

  if (angular_z > max_angle) {
    angular_z = max_angle;
  } else if (angular_z < -max_angle) {
    angular_z = -max_angle;
  }

  if (fabs(angular_z) > min_angle) {
    linear_x_ = (((max_linear_v - min_linear_v) / (min_angle - max_angle)) * (fabs(angular_z) - min_angle)) + max_linear_v;
  }

  if (before_linear_x_ > linear_x_ + STRAIGHT_LINEAR_DECRESE_GAIN2) {
    before_linear_x_ -= STRAIGHT_LINEAR_DECRESE_GAIN2;
  } else if (before_linear_x_ < linear_x_ - STRAIGHT_LINEAR_INCRESE_GAIN2) {
    before_linear_x_ += STRAIGHT_LINEAR_INCRESE_GAIN2;
  } else {
    before_linear_x_ = linear_x_;
  }

  setSpeed(before_linear_x_, angular_z * before_linear_x_ * ANGULAR_LINEAR_RATE3);

  pre_pixel_gap = pixel_gap;
}

bool RobitDriving::imuTurn() {
  double imu_gap, angular_spd;
  imu_gap = fabs(target_imu_ - Vision_msg_.rel_angle);
  if (fabs(imu_gap) > 180) imu_gap = 360 - fabs(imu_gap);
  angular_spd = imu_gap * 0.025 * 2;
  if (target_spd_ < 0) angular_spd *= -1;
  if ((target_spd_ > 0 && angular_spd > target_spd_) || (target_spd_ < 0 && angular_spd < target_spd_)) angular_spd = target_spd_;
  if (fabs(imu_gap) < 3) {
    setSpeed(0.0, 0.0);
    return false;
  } else {
    setSpeed(0.0, angular_spd);
    return true;
  }
}

void RobitDriving::crossMotion() {
  std::cout << "Cross motion: cross_info=" << cross_info_ << ", rel_angle=" << Vision_msg_.rel_angle << std::endl;
  switch (cross_info_) {
    case WAIT:
      std::cout << "Waiting at intersection" << std::endl;
      setSpeed(0.0, 0);
      break;
    case TURN_LEFT:
      std::cout << "Turning left" << std::endl;
      setSpeed(0.04, 0.2);
      break;
    case TURN_RIGHT:
      std::cout << "Turning right" << std::endl;
      setSpeed(0.04, -0.2);
      break;
    case DRIVE:
      std::cout << "Driving straight" << std::endl;
      lineTracing(0.08);
      break;
    case CROSS_END_LEFT:
      std::cout << "Cross end left" << std::endl;
      setSpeed(0.08, 0.3);
      break;
    case CROSS_END_RIGHT:
      std::cout << "Cross end right" << std::endl;
      setSpeed(0.08, -0.3);
      break;
    case CROSS_END:
      std::cout << "Cross end" << std::endl;
      lineTracing(0.15);
      master_msg_.cross_done = true;
      break;
  }
}
void RobitDriving::constructMotion() {
  std::cout << "Construct motion: construct_info=" << construct_info_ << ", rel_angle=" << Vision_msg_.rel_angle << std::endl;
  switch (construct_info_) {
    case SLOW:
      std::cout << "Slowing down" << std::endl;
      lineTracing(0.15);
      break;
    case LEFT__:
      std::cout << "LEFT__" << std::endl;
      lineTracing2(0.13);
      break;
    case LEFT_C1:
      std::cout << "LEFT_C1" << std::endl;
      lineTracing3(0.13);
      break;
      //   case LEFT__:
      //     std::cout << "Constructing left__" << std::endl;
      //     setSpeed(0.5, 0.00);
      //   case LEFT_C1:
      //     std::cout << "Constructing left C1" << std::endl;
      //     setSpeed(0.1, 0.15);
      //     break;
      //   case FRONT_C1:
      //     std::cout << "Constructing front C1" << std::endl;
      //     setSpeed(0.0, 0.15);
      //     break;
      //   case FRONT_C2:
      //     std::cout << "Constructing front C2" << std::endl;
      //     setSpeed(0.1, -0.3);
      //     break;
    case FRONT_C3:
      std::cout << "Constructing front C3" << std::endl;
      lineTracing(0.10);
      break;
    case CONSTRUCT_END:
      std::cout << "Construction end" << std::endl;
      master_msg_.construct_done = true;
      break;
  }
}

void RobitDriving::parkingMotion() {
  std::cout << "Parking motion: parking_info=" << parking_info_ << ", rel_angle=" << Vision_msg_.rel_angle << std::endl;
  switch (parking_info_) {
    case GO_P1:
      std::cout << "Parking detect_1" << std::endl;
      lineTracing(0.10);  //
      break;
    case GO_P2:
      std::cout << "Parking detect_2" << std::endl;
      lineTracing(0.1);
      break;
    case LEFT_P1:
      std::cout << "Parking left" << std::endl;
      setSpeed(0.1, 0.1);
      break;
    case GO_P3:
      std::cout << "Parking linetracing" << std::endl;
      lineTracing(0.18);
      break;
    case TURN2L:
      std::cout << "Parking turn left" << std::endl;
      setSpeed(0.0, -0.3);
      break;
    case TURN2R:
      std::cout << "Parking turn right" << std::endl;
      setSpeed(0.0, 0.3);
      break;
    case BACK:
      std::cout << "Parking back" << std::endl;
      setSpeed(-0.1, 0.0);
      break;
    case TURN3L:
      std::cout << "Parking turn left" << std::endl;
      setSpeed(0.1, -0.5);
      break;
    case TURN3R:
      std::cout << "Parking turn right" << std::endl;
      setSpeed(0.1, 0.5);
      break;
    case GO_P4:
      std::cout << "Parking linetracing" << std::endl;
      lineTracing(0.18);
      break;
    case LEFT_P2:
      std::cout << "Parking left" << std::endl;
      setSpeed(0, 0);
      // setSpeed(0.1, 0.5);
      break;
    case PARKING_END:
      std::cout << "Parking end" << std::endl;
      master_msg_.parking_done = true;

    default:
      break;
  }
}

}  // namespace robit_master