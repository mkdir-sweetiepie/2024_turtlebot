#include "../include/robit_vision/vision.hpp"

namespace vision_colors {
const cv::Scalar Red(0, 0, 255);
const cv::Scalar Blue(255, 0, 0);
const cv::Scalar Green(0, 255, 0);
const cv::Scalar Yellow(0, 255, 255);
const cv::Scalar CPL(255, 153, 153);
const cv::Scalar Pink(180, 105, 255);
const cv::Scalar Skyblue(255, 153, 0);
const cv::Scalar Mint(208, 224, 64);
}  // namespace vision_colors

robit_msgs::msg::VisionMsg Vision::Vision_msg;
robit_msgs::msg::MasterMsg Vision::Master_msg;
bool Vision::led[6] = {false, false, false, false, false, false};
int Vision::psd[PSD_SENSORS] = {0};
int Vision::count_fps = 0, Vision::now_fps = 0;
bool Vision::timer_ctrl = false;
int Vision::timer_cnt = 0;
int Vision::traffic_value[6] = {0}, Vision::parking_value[6] = {0}, Vision::gatebar_value[6] = {0}, Vision::bluesign_value[6] = {0};
bool Vision::rel_ctrl = false, Vision::rel_round = false;
int Vision::direction_angle = 0, Vision::rel_zero_angle = 0, Vision::rel_angle = 0;
int Vision::start_imu[6] = {90, 0, 180, 0, 180, 0};
double Vision::rel_angle_ratio = 0;
int Vision::now_mission = 1;
int Vision::mission_sequence[Vision::MISSION_COUNT] = {1, 2, 3, 4, 5, 6};

// cross
bool Vision::cross_detect = false;
int Vision::cross_condition = 0;
bool Vision::cross_start = false;
// bool Vision::direction_flag = false;
int direction_counter = 0;
bool Vision::cross_step[5] = {false, false, false, false, false};
// CrossDirection Vision::cross_direction = CrossDirection::NONE;
unsigned long Vision::beforeMax = 0;
unsigned long Vision::nowMax = 0;
unsigned long Vision::nowMax_i = 0;

bool Vision::direction = false;
bool Vision::follow_bluesign = false;  //= true;//
int Vision::left_count = 0;
int Vision::right_count = 0;
bool Vision::process_flag[3] = {false, false, false};
bool Vision::sign_once = false;
bool Vision::sign_twice = false;
bool Vision::cross_flag = false;
// construct
bool Vision::construct_detect = false;
int Vision::construct_condition = 0;
bool Vision::construct_start = false;
bool Vision::construct_step[6] = {false, false, false, false, false, false};

// parking
bool Vision::parking_detect = false;
int Vision::parking_condition = 0;
bool Vision::parking_step[9] = {false, false, false, false, false, false, false, false, false};
bool Vision::rotate = false;
int Vision::cnt = 0;
int Vision::cnt2 = 0;
bool Vision::ctrl_timer = false;

// zigzag
bool Vision::zigzag_detect = false;
bool Vision::zigzag_condition = false;

// gatebar
bool Vision::gatebar_detect = false;
bool Vision::gatebar_start = false;

int Vision::just_before_tunnel_num = 4;
int Vision::white_box_cnt = 0;
int Vision::white_line_blob_y = 0, Vision::white_line_blob_x = 0;
unsigned long Vision::escape_standard = 0;

int Vision::traffic_action = 0;

int VisionLine::r_line_xpos = 0, VisionLine::l_line_xpos = 0;
bool VisionLine::r_line_detect = false, VisionLine::l_line_detect = false, VisionLine::y_line_detect = false, VisionLine::w_line_detect = false;
double VisionLine::white_line_deg = 0.0, VisionLine::yellow_line_deg = 0.0;
int VisionLine::yellow_value[6] = {0}, VisionLine::white_value[6] = {0};
int VisionLine::Canny_value[2] = {0};
int VisionLine::cross_yellow_y = 0;

Vision::Vision(QObject *parent) : QObject(parent) {}

Vision::~Vision() {}

void Vision::START(const cv::Mat &input_img) {
  Perspective_View(input_img, Perspective_img);
  VisionLine line(Perspective_img, parking_detect, 1);
  Perspective_View2(input_img, Perspective_img2);
  VisionLine line2(Perspective_img2, parking_detect, 2);
  Raw_image = input_img.clone();

  if (!Master_msg.traffic_done) {
    finding_traffic_lights(Raw_image, const_cast<cv::Mat &>(input_img));
  } else if (now_mission == mission_sequence[M_CROSS] && !Master_msg.cross_done) {
    Cross_Process();
  } else if (now_mission == mission_sequence[M_CONSTRUCT] && !Master_msg.construct_done) {
    Construct_Process();
  } else if (now_mission == mission_sequence[M_PARKING] && !Master_msg.parking_done) {
    Parking_Process();
  } else if (now_mission == mission_sequence[M_ZIGZAG]) {
    Zigzag_Process();
  } else if (now_mission == mission_sequence[M_GATEBAR]) {
    Finding_Gatebar(Raw_image, const_cast<cv::Mat &>(input_img));
  }

  Update_Message();

  Q_EMIT perspective_callback(Perspective_img);
  Q_EMIT perspective_callback2(Perspective_img2);
  Q_EMIT white_line_callback(line.white_line);
  Q_EMIT yellow_line_callback(line.yellow_line);
  Q_EMIT mission_callback();
}

void Vision::Update_Message() {
  Vision_msg.r_diff_pixel = VisionLine::r_line_xpos;
  Vision_msg.l_diff_pixel = VisionLine::l_line_xpos;
  Vision_msg.r_angle = static_cast<int>(VisionLine::white_line_deg);
  Vision_msg.l_angle = static_cast<int>(VisionLine::yellow_line_deg);
  Vision_msg.r_line_info = VisionLine::r_line_detect;
  Vision_msg.l_line_info = VisionLine::l_line_detect;

  Vision_msg.traffic = traffic_action;
  Vision_msg.cross_detect = cross_detect;
  Vision_msg.cross_info = cross_condition;
  Vision_msg.construct_detect = construct_detect;
  Vision_msg.construct_info = construct_condition;
  Vision_msg.parking_detect = parking_detect;
  Vision_msg.parking_info = parking_condition;
  Vision_msg.zigzag_detect = zigzag_detect;
  Vision_msg.zigzag_info = zigzag_condition;
  Vision_msg.gatebar_detect = gatebar_detect;
  Vision_msg.gatebar_info = gatebar_start;
  Vision_msg.just_before_tunnel_num = just_before_tunnel_num;

  Vision_msg.rel_angle = rel_angle;
  Vision_msg.rel_angle_ratio = rel_angle_ratio;

  for (int i = 0; i < MISSION_COUNT; ++i) {
    Vision_msg.mission_sequence[i] = mission_sequence[i];
  }
}

void Vision::update_parameter(const std::shared_ptr<const robit_msgs::msg::MasterMsg> &master_data) {
  Master_msg = *master_data;
  if (Master_msg.led_init) {
    for (int i = 0; i < 6; ++i) {
      led[i] = true;
    }
  }
}

void Vision::Perspective_View(const cv::Mat &input_img, cv::Mat &output_img) {
  std::array<cv::Point2f, 4> inputQuad = {cv::Point2f(60, 480 - 120), cv::Point2f(640 - 60, 480 - 120), cv::Point2f(640 - 10, 480 - 60), cv::Point2f(10, 480 - 60)};

  std::array<cv::Point2f, 4> outputQuad = {cv::Point2f(0, 0), cv::Point2f(RAW_X, 0), cv::Point2f(RAW_X, RAW_Y), cv::Point2f(0, RAW_Y)};

  cv::Mat lambda = cv::getPerspectiveTransform(inputQuad.data(), outputQuad.data());
  cv::warpPerspective(input_img, output_img, lambda, cv::Size(RAW_X, RAW_Y));

  for (int i = 0; i < 4; ++i) {
    cv::circle(const_cast<cv::Mat &>(input_img), inputQuad[i], 3, vision_colors::Mint, -1);
    cv::putText(const_cast<cv::Mat &>(input_img), std::to_string(i), inputQuad[i], 1, 2, vision_colors::Mint, 1, 8);
  }

  cv::line(const_cast<cv::Mat &>(input_img), inputQuad[0], inputQuad[3], vision_colors::Mint, 2, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), inputQuad[1], inputQuad[2], vision_colors::Mint, 2, cv::LINE_AA);
}

void Vision::Perspective_View2(const cv::Mat &input_img, cv::Mat &output_img) {
  std::array<cv::Point2f, 4> inputQuad = {cv::Point2f(150, 190), cv::Point2f(640 - 150, 190), cv::Point2f(640 - 150, 265), cv::Point2f(150, 265)};

  std::array<cv::Point2f, 4> outputQuad = {cv::Point2f(0, 0), cv::Point2f(RAW_X, 0), cv::Point2f(RAW_X, RAW_Y), cv::Point2f(0, RAW_Y)};

  cv::Mat lambda = cv::getPerspectiveTransform(inputQuad.data(), outputQuad.data());
  cv::warpPerspective(input_img, output_img, lambda, cv::Size(RAW_X, RAW_Y));

  for (const auto &point : inputQuad) {
    cv::circle(const_cast<cv::Mat &>(input_img), point, 3, vision_colors::Green, -1);
  }

  cv::line(const_cast<cv::Mat &>(input_img), inputQuad[0], inputQuad[3], vision_colors::Pink, 2, cv::LINE_AA);
  cv::line(const_cast<cv::Mat &>(input_img), inputQuad[1], inputQuad[2], vision_colors::Pink, 2, cv::LINE_AA);
}

void Vision::Change_to_Binary(const cv::Mat &input_img, cv::Mat &output_img, const int value[6], bool flag) {
  cv::Mat mask1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat mask2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));

  cv::medianBlur(input_img, output_img, 9);
  cv::GaussianBlur(output_img, output_img, cv::Size(15, 15), 2.0);
  cv::cvtColor(output_img, output_img, cv::COLOR_RGB2HSV);

  cv::inRange(output_img, cv::Scalar(value[3], value[4], value[5]), cv::Scalar(value[0], value[1], value[2]), output_img);

  if (!flag) {
    cv::erode(output_img, output_img, mask1, cv::Point(-1, -1), 2);
    cv::dilate(output_img, output_img, mask1, cv::Point(-1, -1), 2);
  } else {
    cv::erode(output_img, output_img, mask1, cv::Point(-1, -1), 2);
    cv::dilate(output_img, output_img, mask2, cv::Point(-1, -1), 2);
  }
}

void Vision::finding_traffic_lights(const cv::Mat &raw_image, cv::Mat &draw_image) {
  cv::Mat Traffic_Sign_img = raw_image(cv::Rect(380, 170, 80, 80)).clone();  // 640 - 150, 265
  Change_to_Binary(Traffic_Sign_img, Traffic_Sign_img, traffic_value, false);
  Q_EMIT traffic_callback(Traffic_Sign_img);
  cv::rectangle(draw_image, cv::Rect(380, 170, 80, 80), vision_colors::Green, 2);
  VisionLabeling Traffic_labeling(Traffic_Sign_img, 10);
  Traffic_labeling.do_labeling();
  Traffic_labeling.draw_maximum_label_rect(draw_image);

  if (!Traffic_labeling.blobs.empty()) {
    const auto &max_blob = Traffic_labeling.blobs[Traffic_labeling.max_num];
    int x = max_blob.x + (max_blob.width / 2) + 380;
    int y = max_blob.y + (max_blob.height / 2) + 170;
    int radius = max_blob.width / 2;
    cv::circle(draw_image, cv::Point(x, y), radius, vision_colors::Red, 3, 8, 0);
    traffic_action = 1;
    setLed(false, false, false, false, false, false);
  }
  // else {
  //   traffic_action = 0;
  // }
}

void Vision::Cross_Process() {
  static CrossCondition cross_condition = CrossCondition::APPROACHING;
  auto update_led = [this](int led_index) { led[led_index] = false; };
  static bool detection_attempted = false;
  static int timeout_counter = 0;
  const int TIMEOUT_LIMIT = 100;

  if (!rel_ctrl) {
    rel_zero_angle = start_imu[M_CROSS];
    rel_ctrl = true;
    detection_attempted = false;
    timeout_counter = 0;
  }

  switch (cross_condition) {
    case CrossCondition::APPROACHING:
      if (rel_round && (rel_angle >= 75 && rel_angle < 90)) {
        // 이각도에서 cross 시작
        cross_detect = true;

        update_led(0);
        cross_start = true;
        std::cout << "Yellow sign detected" << std::endl;
        cross_condition = CrossCondition::YELLOW_SIGN_PASSED;
      }
      if (rel_round && (rel_angle >= 75 && rel_angle < 90)) {
        // 표지판 인식 시도
        if (!detection_attempted) {
          cross_start = true;          // 블루사인 인식 활성화
          detection_attempted = true;  // 인식 시도 표시
          std::cout << "Starting blue sign detection" << std::endl;
        }
        // 인식 시도 후에도 방향이 결정되지 않은 경우에만 타이머 증가
        else if (!sign_condition[left] && !sign_condition[right]) {
          timeout_counter++;
          if (timeout_counter > TIMEOUT_LIMIT) {
            std::cout << "Detection timeout - defaulting to right turn" << std::endl;
            sign_condition[right] = true;  // 기본값으로 우회전 설정
            direction = right;
            cross_detect = true;
            cross_condition = CrossCondition::TURN_RIGHT;
            timeout_counter = 0;
          }
        }
      }
      break;

    case CrossCondition::YELLOW_SIGN_PASSED:
      if ((rel_angle >= 75 && rel_angle < 90)) {
        update_led(1);
        cross_step[0] = true;
        std::cout << "Ready for direction detection" << std::endl;
        cross_condition = CrossCondition::BLUE_SIGN_DETECTING;
        Vision::cross_condition = 2;
        detection_attempted = false;  // 다음 인식을 위해 리셋
        timeout_counter = 0;
      }
      break;

    case CrossCondition::BLUE_SIGN_DETECTING:

      update_led(2);
      if (sign_condition[right]) {
        cross_condition = CrossCondition::TURN_RIGHT;
        cross_step[1] = true;
        Vision::cross_condition = 4;
      } else if (sign_condition[left]) {
        cross_condition = CrossCondition::TURN_LEFT;
        cross_step[1] = true;
        Vision::cross_condition = 3;
      }

      // CrossDirection current_direction = cross_direction.load();
      // if (current_direction == CrossDirection::LEFT) {
      //   cross_condition = CrossCondition::TURN_LEFT;
      //   cross_step[1] = true;
      // } else if (current_direction == CrossDirection::RIGHT) {
      //   cross_condition = CrossCondition::TURN_RIGHT;
      //   cross_step[1] = true;
      // }
      // std::cout << "Direction set in Cross_Process: " << (current_direction == CrossDirection::LEFT ? "LEFT" : "RIGHT") << std::endl;

      break;

    case CrossCondition::TURN_LEFT:
    case CrossCondition::TURN_RIGHT:
      cross_step[0] = false;
      cross_step[1] = false;

      if ((rel_angle > 140) || (rel_angle <= 30)) {
        update_led(3);

        cross_condition = CrossCondition::DRIVE;
        cross_step[2] = true;
      }
      break;
    case CrossCondition::DRIVE:
      std::cout << "Drive" << std::endl;
      cnt_cross++;
      std::cout << "cnt_cross: " << cnt_cross << std::endl;
      if (cnt_cross > 50) {
        if ((sign_condition[left] && ((rel_angle < 60 && rel_angle > 1) && (!VisionLine::l_line_detect))) ||
            (sign_condition[right] && ((rel_angle >= 150 && rel_angle <= 190) && (!VisionLine::r_line_detect)))) {
          update_led(4);
          cross_step[3] = true;

          if (sign_condition[right])
            cross_condition = CrossCondition::CROSS_END_RIGHT;
          else if (sign_condition[left])
            cross_condition = CrossCondition::CROSS_END_LEFT;
        }
      }
      break;
    case CrossCondition::CROSS_END_LEFT:
    case CrossCondition::CROSS_END_RIGHT:
      cnt_cross = 0;
      if (rel_angle >= 70 && rel_angle <= 100) {
        cross_step[4] = true;  // cross end
        cross_condition = CrossCondition::CROSS_END;
      }
      break;
    case CrossCondition::CROSS_END:
      update_led(5);
      now_mission++;
      std::cout << "Cross completed" << std::endl;
      break;
  }
}

void Vision::Follow_bluesign(cv::Mat &input_img) {
  if (!cross_start) return;

  const int blue_img_y = 0;
  bluesign_img = input_img(cv::Rect(200, 100, 320, 180)).clone();
  Change_to_Binary(bluesign_img, bluesign_img, bluesign_value, 0);
  gray_img = bluesign_img.clone();
  Q_EMIT bluesignDetected(bluesign_img);

  VisionLabeling bluesign_labeling(bluesign_img, 600);
  bluesign_labeling.do_labeling();
  Vision::nowMax = 0;
  Vision::nowMax_i = 0;

  if (bluesign_labeling.blobs.size() > 0) {
    bluesign_detect = true;

    // 가장 큰 blob 찾기 및 처리
    Vision::beforeMax = 0;
    for (unsigned long i = 0; i < bluesign_labeling.blobs.size(); i++) {
      if (beforeMax < (unsigned long)(bluesign_labeling.blobs[i].width * bluesign_labeling.blobs[i].height)) {
        nowMax = (unsigned long)(bluesign_labeling.blobs[i].width * bluesign_labeling.blobs[i].height);
        nowMax_i = i;
      }
      beforeMax = nowMax;
    }

    // 표지판이 충분히 크고 아직 방향이 결정되지 않았을 때만 처리
    if (!cross_detect_once && bluesign_labeling.blobs[nowMax_i].width > 30) {
      int Lsum = 0, Rsum = 0;
      double blue_x = bluesign_labeling.blobs[nowMax_i].x + bluesign_labeling.blobs[nowMax_i].width / 2;
      int start_y = bluesign_labeling.blobs[nowMax_i].y + bluesign_labeling.blobs[nowMax_i].height * 2 / 5;
      int end_y = bluesign_labeling.blobs[nowMax_i].y + bluesign_labeling.blobs[nowMax_i].height * 3 / 4;

      // 화살표 검출
      for (int j = start_y; j < end_y; j++) {
        for (int i = blue_x - bluesign_labeling.blobs[nowMax_i].width / 3; i < blue_x; i++) {
          if (gray_img.at<uchar>(j, i) < 150) Lsum++;
        }
        for (int i = blue_x; i < blue_x + bluesign_labeling.blobs[nowMax_i].width / 3; i++) {
          if (gray_img.at<uchar>(j, i) < 150) Rsum++;
        }
      }

      // 결과 디버깅 출력
      std::cout << "Lsum: " << Lsum << " Rsum: " << Rsum << std::endl;

      if (Lsum < Rsum * 0.8) {
        left_count++;
        if (right_count > 0) right_count--;
      } else if (Rsum < Lsum * 0.8) {
        right_count++;
        if (left_count > 0) left_count--;
      }

      if (left_count > 2 || right_count > 3) {
        if (!process_flag[0]) {
          cross_detect = true;
          cross_detect_once = true;
          if (left_count > right_count) {
            sign_condition[left] = true;
            direction = left;
            std::cout << "Left sign detected" << std::endl;
          } else {
            sign_condition[right] = true;
            direction = right;
            std::cout << "Right sign detected" << std::endl;
          }
          process_flag[0] = true;
          cross_start = false;  // 방향 결정되면 인식 중단
        }
      }
    }
  }
}


void Vision::Construct_Process() {
  static ConstructCondition construct_condition = ConstructCondition::SLOW;

  auto update_led = [this](int led_index) { led[led_index] = true; };

  if (rel_angle >= 260 && rel_angle <= 280) {
    rel_ctrl = false;
    if (!rel_ctrl) {
      // 기준 IMU(start_imu)를 0도로 함.
      rel_zero_angle = start_imu[M_CONSTRUCT];
      rel_ctrl = true;
      update_led(0);

      construct_start = true;
      std::cout << "Construct slow" << std::endl;
      construct_detect = true;
    }
  }
  if (construct_detect) {
    switch (construct_condition) {
      case ConstructCondition::SLOW:
        // if (!(VisionLine::l_line_detect)) {
        // cnt++;
        // std::cout << cnt << std::endl;
        // if (cnt > 100) {
        if (psd[2] > 600) {
          update_led(1);
          construct_step[0] = true;
          std::cout << "Construct left __" << std::endl;
          construct_condition = ConstructCondition::LEFT__;
          //   } else {
          //     update_led(2);
          //     construct_step[1] = true;
          //     std::cout << "Construct left __" << std::endl;
          //     construct_condition = ConstructCondition::LEFT_C1;
          //   }
          // }
        }
        break;

      case ConstructCondition::LEFT__:
        if (rel_angle > 140 && rel_angle < 190) {
          update_led(2);
          // std::cout << "Construct front C3" << std::endl;
          construct_step[4] = true;
          construct_condition = ConstructCondition::FRONT_C3;
        }
        break;
      case ConstructCondition::LEFT_C1:
        if (rel_angle > 140 && rel_angle < 190) {
          update_led(3);
          // std::cout << "Construct front C3" << std::endl;
          construct_step[4] = true;
          construct_condition = ConstructCondition::FRONT_C3;
        }
        break;
      //   if (psd[1] > 300 && psd[2] > 300) {
      //     update_led(1);
      //     construct_step[0] = true;
      //     std::cout << "Construct left __" << std::endl;
      //     construct_condition = ConstructCondition::LEFT__;
      //   }
      //   break;
      // case ConstructCondition::LEFT__:
      //   if (rel_angle > 20 && rel_angle < 130) {
      //     update_led(2);
      //     construct_step[1] = true;
      //     std::cout << "Construct left __" << std::endl;
      //     construct_condition = ConstructCondition::LEFT_C1;
      //   }
      //   break;
      // case ConstructCondition::LEFT_C1:
      //   if (psd[1] > 300) {
      //     update_led(3);
      //     construct_step[2] = true;
      //     std::cout << "Construct left C1" << std::endl;
      //     construct_condition = ConstructCondition::FRONT_C1;
      //   }
      //   break;
      // case ConstructCondition::FRONT_C1:
      //   if (rel_angle > 140 && rel_angle < 160) {
      //     update_led(3);
      //     construct_step[3] = true;
      //     std::cout << "Construct front C1" << std::endl;
      //     construct_condition = ConstructCondition::FRONT_C2;
      //   }
      //   break;
      // case ConstructCondition::FRONT_C2:
      //   if (rel_angle > 40 && rel_angle < 70) {
      //     update_led(4);
      //     construct_step[4] = true;
      //     std::cout << "Construct front C2" << std::endl;
      //     construct_condition = ConstructCondition::FRONT_C3;
      //   }
      //   break;
      case ConstructCondition::FRONT_C3:
        if (VisionLine::l_line_detect && VisionLine::r_line_detect) {
          update_led(4);
          std::cout << "Construct front C3" << std::endl;
          construct_step[5] = true;
          construct_condition = ConstructCondition::CONSTRUCT_END;
        }
        break;
      case ConstructCondition::CONSTRUCT_END:
        update_led(5);
        std::cout << "Construct end" << std::endl;
        now_mission++;
        break;
    }
  }
}

void Vision::Parking_Process() {
  auto update_led = [this](int led_index) { led[led_index] = false; };
  static ParkingCondition parking_condition = ParkingCondition::GO_P1;

  cnt++;
  if (rel_angle >= 150 && rel_angle <= 200 && !parking_detect && cnt > 80) {
    rel_ctrl = false;
    if (!rel_ctrl) {
      rel_zero_angle = start_imu[M_PARKING];
      rel_ctrl = true;
      update_led(0);
      parking_detect = true;
    }
    cnt = 0;
  } else if (parking_detect) {
    switch (parking_condition) {
      case ParkingCondition::GO_P1:
        if (rel_round && !(VisionLine::l_line_detect)) {
          update_led(1);
          parking_step[1] = true;
          std::cout << "parking detect_1" << std::endl;
          parking_condition = ParkingCondition::LEFT_P1;
        }
        break;
      case ParkingCondition::LEFT_P1:
        if ((parking_condition == ParkingCondition::LEFT_P1) && ((rel_angle >= 70) && (rel_angle <= 90))) {
          update_led(2);
          parking_step[2] = true;
          std ::cout << "Parking left" << std::endl;
          parking_condition = ParkingCondition::GO_P3;
        }
        break;
      case ParkingCondition::GO_P3:
        if ((parking_condition == ParkingCondition::GO_P3) && (psd[2] > 400 || psd[0] > 400)) {
          parking_step[3] = true;
          std::cout << "Parking zone" << std::endl;
          parking_condition = ParkingCondition::DETECT;
          rotate = (psd[0] > 300);  // 오른쪽에 있음
        }
        break;
      case ParkingCondition::DETECT:
        parking_condition = rotate ? ParkingCondition::TURN2L : ParkingCondition::TURN2R;
        break;

      case ParkingCondition::TURN2L:
      case ParkingCondition::TURN2R:
        if ((parking_condition == ParkingCondition::TURN2L && psd[1] > 400 && rel_angle <= 10) || (parking_condition == ParkingCondition::TURN2R && psd[1] > 400 && rel_angle >= 150)) {
          update_led(2);
          std::cout << "Parking turn" << std::endl;
          parking_step[4] = true;
          parking_condition = ParkingCondition::BACK;
        }
        break;
      case ParkingCondition::BACK:
        cnt2++;
        if (cnt2 > 38) {
          update_led(3);
          std::cout << "Parking back" << std::endl;
          parking_step[5] = true;
          parking_condition = rotate ? ParkingCondition::TURN3L : ParkingCondition::TURN3R;
        }
        break;
      case ParkingCondition::TURN3L:
      case ParkingCondition::TURN3R:
        if ((parking_condition == ParkingCondition::TURN3L && rel_angle <= 270 && rel_angle > 200) || (parking_condition == ParkingCondition::TURN3R && rel_angle >= 260)) {
          update_led(4);
          std::cout << "Parking turn left or right" << std::endl;
          parking_step[6] = true;
          parking_condition = ParkingCondition::GO_P4;
        }

        break;
      case ParkingCondition::GO_P4:
        if (!VisionLine::l_line_detect && !VisionLine::r_line_detect) {
          update_led(5);
          parking_step[7] = true;
          std::cout << "Parking trun right" << std::endl;
          parking_condition = ParkingCondition::LEFT_P2;
        }
        break;
      case ParkingCondition::LEFT_P2:
        if (rel_angle >= 330 || rel_angle <= 20) {
          parking_step[8] = true;
          std::cout << "Parking left" << std::endl;
          parking_condition = ParkingCondition::PARKING_END;
        }
        break;
      case ParkingCondition::PARKING_END:
        std::cout << "Parking end" << std::endl;
        now_mission++;
        break;
    }
  }
  // if (parking_detect) {
  //   switch (parking_condition) {
  //     case ParkingCondition::GO_P1:
  //       if (rel_round && !(VisionLine::l_line_detect)) {
  //         parking_step[0] = true;
  //         std::cout << "parking detect_1" << std::endl;
  //         parking_condition = ParkingCondition::LEFT_P1;
  //       }
  //       break;
  //     case ParkingCondition::GO_P2:
  //       parking_detect = true;
  //       timer_cnt++;
  //       if ((parking_condition == ParkingCondition::GO_P2 && !(VisionLine::l_line_detect) && timer_cnt > 20)) {
  //         update_led(1);
  //         std::cout << "parking detect_2" << std::endl;
  //         parking_step[1] = true;
  //         if (cnt == 1)
  //           parking_condition = ParkingCondition::GO_P1;
  //         else
  //           parking_condition = ParkingCondition::LEFT_P1;
  //         timer_cnt = 0;
  //       }
  //       break;
  //     case ParkingCondition::LEFT_P1:
  //       if ((parking_condition == ParkingCondition::LEFT_P1) && ((rel_angle >= 60) && (rel_angle <= 80))) {
  //         update_led(1);
  //         parking_step[2] = true;
  //         std ::cout << "Parking left" << std::endl;
  //         parking_condition = ParkingCondition::GO_P3;
  //       }
  //       break;
  //     case ParkingCondition::GO_P3:
  //       if ((parking_condition == ParkingCondition::GO_P3) && (psd[2] > 400 || psd[0] > 400)) {
  //         parking_step[3] = true;
  //         std::cout << "Parking zone" << std::endl;
  //         parking_condition = ParkingCondition::DETECT;
  //         rotate = (psd[0] > 300);  // 오른쪽에 있음
  //       }
  //       break;
  //     case ParkingCondition::DETECT:
  //       parking_condition = rotate ? ParkingCondition::TURN2L : ParkingCondition::TURN2R;
  //       break;

  //     case ParkingCondition::TURN2L:
  //     case ParkingCondition::TURN2R:
  //       if ((parking_condition == ParkingCondition::TURN2L && psd[1] > 300 && (rel_angle > 160) || (parking_condition == ParkingCondition::TURN2R && psd[1] > 300) && (rel_angle < 20))) {
  //         update_led(2);
  //         std::cout << "Parking turn" << std::endl;
  //         parking_step[4] = true;
  //         parking_condition = ParkingCondition::BACK;
  //       }
  //       break;
  //     case ParkingCondition::BACK:
  //       if (psd[1] < 100) {
  //         update_led(3);
  //         std::cout << "Parking back" << std::endl;
  //         parking_step[5] = true;
  //         parking_condition = rotate ? ParkingCondition::TURN3L : ParkingCondition::TURN3R;
  //       }
  //       break;
  //     case ParkingCondition::TURN3L:
  //     case ParkingCondition::TURN3R:
  //       if (VisionLine::l_line_detect || VisionLine::r_line_detect) {
  //         parking_condition = ParkingCondition::GO_P4;
  //         update_led(4);
  //         std::cout << "Parking turn left" << std::endl;
  //         parking_step[6] = true;
  //       }
  //       break;
  //     case ParkingCondition::GO_P4:
  //       if (!VisionLine::l_line_detect && !VisionLine::r_line_detect) {
  //         update_led(5);
  //         parking_step[7] = true;
  //         std::cout << "Parking trun right" << std::endl;
  //         parking_condition = ParkingCondition::LEFT_P2;
  //       }
  //       break;
  //     case ParkingCondition::LEFT_P2:
  //       if (rel_angle >= 330 || rel_angle <= 10) {
  //         parking_step[9] = true;
  //         std::cout << "Parking left" << std::endl;
  //         parking_condition = ParkingCondition::PARKING_END;
  //       }
  //       break;
  //     case ParkingCondition::PARKING_END:
  //       std::cout << "Parking end" << std::endl;
  //       now_mission++;
  //       break;
  //   }
  // }
}
void Vision::Zigzag_Process() {
  zigzag_detect = true;
  std::cout << "Zigzag mission start!" << std::endl;
  if (rel_angle >= 170 && rel_angle <= 190) {
    now_mission++;
    zigzag_condition = 1;
    std::cout << "Zigzag mission complete!" << std::endl;
  }
}

void Vision::Finding_Gatebar(const cv::Mat &raw_image, cv::Mat &input_img) {
  cv::Mat Gatebar_img = raw_image(cv::Rect(200, 200, 320, 150)).clone();
  Change_to_Binary(Gatebar_img, Gatebar_img, gatebar_value, false);
  cv::Mat resized_display;
  cv::resize(Gatebar_img, resized_display, cv::Size(320, 240));
  Q_EMIT gatebar_callback(Gatebar_img);
  cv::rectangle(input_img, cv::Rect(200, 200, 320, 150), vision_colors::Blue, 2);
  VisionLabeling Gatebar_labeling(Gatebar_img, 350);
  Gatebar_labeling.do_labeling();

  if (!Gatebar_labeling.blobs.empty()) {
    int gate = 0;
    int max_x = Gatebar_labeling.blobs[0].x + 200;
    int min_x = Gatebar_labeling.blobs[0].x + 200;
    int max_num = 0, min_num = 0;

    for (size_t i = 0; i < Gatebar_labeling.blobs.size(); i++) {
      if (Gatebar_labeling.blobs[i].width > 16 && Gatebar_labeling.blobs[i].height > 16) {
        gate++;
        cv::Rect blob_rect = Gatebar_labeling.blobs[i];
        blob_rect.x += 200;
        blob_rect.y += 200;
        cv::rectangle(input_img, blob_rect, vision_colors::Red, 2);

        if (Gatebar_labeling.blobs[i].x > max_x - 200) {
          max_x = Gatebar_labeling.blobs[i].x + 200;
          max_num = i;
        }
        if (Gatebar_labeling.blobs[i].x < min_x - 200) {
          min_x = Gatebar_labeling.blobs[i].x + 200;
          min_num = i;
        }
      }
    }

    if (Gatebar_labeling.blobs.size() >= 3 && gate >= 3 && !gatebar_detect) {
      // 처음 감지됐을 때만 LED ON과 detect true
      setLed(false, false, false, false, false, false);
      gatebar_detect = true;
      std::cout << "Gatebar detected!" << std::endl;
    }
  } else if (Gatebar_labeling.blobs.empty() && gatebar_detect) {  // 블랍이 사라지고, 이전에 감지된 적이 있다면
    setLed(true, true, true, true, true, true);
    gatebar_start = true;
    now_mission++;
    std::cout << "Gatebar mission complete!" << std::endl;
  }
}

void Vision::setLed(bool _led1, bool _led2, bool _led3, bool _led4, bool _led5, bool _led6) {
  led[0] = !_led1;
  led[1] = !_led2;
  led[2] = !_led3;
  led[3] = !_led4;
  led[4] = !_led5;
  led[5] = !_led6;
}

VisionLine::VisionLine(const cv::Mat &perspective_img, bool is_parking, int num) {
  cv::Mat grey_img, edge_img;
  cv::Mat twoLineToOneLineEdgeHorizontal = cv::Mat::zeros(perspective_img.size(), CV_8UC1);
  cv::Mat twoLineToOneLineEdgeVertical = cv::Mat::zeros(perspective_img.size(), CV_8UC1);

  cv::GaussianBlur(perspective_img, grey_img, cv::Size(15, 15), 2.0);
  cv::cvtColor(grey_img, grey_img, cv::COLOR_BGR2GRAY);
  cv::Canny(grey_img, edge_img, Canny_value[0], Canny_value[1], 3);

  MakeHorizontalOneLine(edge_img, twoLineToOneLineEdgeHorizontal);
  MakeVerticalOneLine(edge_img, twoLineToOneLineEdgeVertical);

  cv::Mat morphDilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat morphDilateVer = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 3));
  cv::Mat morphDilateHor = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 2));

  cv::morphologyEx(twoLineToOneLineEdgeHorizontal, twoLineToOneLineEdgeHorizontal, cv::MORPH_DILATE, morphDilateHor);
  cv::morphologyEx(twoLineToOneLineEdgeVertical, twoLineToOneLineEdgeVertical, cv::MORPH_DILATE, morphDilateVer);

  removeSmallEdgeComponents(twoLineToOneLineEdgeHorizontal, 15);
  removeSmallEdgeComponents(twoLineToOneLineEdgeVertical, 15);

  cv::Mat leftLine, rightLine, twoLineToOneLineEdgeSum;
  twoLineToOneLineEdgeSum = twoLineToOneLineEdgeHorizontal + twoLineToOneLineEdgeVertical;
  cv::morphologyEx(twoLineToOneLineEdgeSum, twoLineToOneLineEdgeSum, cv::MORPH_DILATE, morphDilate);
  removeSmallEdgeComponents(twoLineToOneLineEdgeSum, 30);

  Judgement_Line(perspective_img, twoLineToOneLineEdgeSum, leftLine, rightLine, yellow_value, white_value, is_parking);

  if (num == 1)
    Extract_Line(perspective_img, leftLine, rightLine);
  else
    Extract_Line_Color(perspective_img, leftLine, rightLine);
}

void VisionLine::MakeHorizontalOneLine(const cv::Mat &src, cv::Mat &dst, int minLineWidth, int maxLineWidth) {
  if (dst.empty()) {
    dst = cv::Mat::zeros(src.size(), CV_8UC1);
  }

  for (int y = 1; y < src.rows - 1; y++) {
    bool edgeStarted = false;
    int lineStartX = 0;
    int lineWidth = 0;

    for (int x = 1; x < src.cols - 1; x++) {
      if (!edgeStarted && src.at<uchar>(y, x) != 0) {
        lineStartX = x;
        edgeStarted = true;
      } else if (edgeStarted) {
        lineWidth = x - lineStartX;

        if (lineWidth > maxLineWidth) {
          edgeStarted = false;
        } else if (lineWidth >= minLineWidth && src.at<uchar>(y, x) != 0) {
          dst.at<uchar>(y, lineStartX + lineWidth / 2) = 255;
          edgeStarted = false;
        }
      }
    }
  }
}

void VisionLine::MakeVerticalOneLine(const cv::Mat &src, cv::Mat &dst, int minLineWidth, int maxLineWidth) {
  if (dst.empty()) dst = cv::Mat::zeros(src.size(), CV_8UC1);

  for (int x = 1; x < src.cols - 1; x++) {
    bool edgeStarted = false;
    int lineStartY = 0;
    int lineWidth = 0;

    for (int y = 1; y < src.rows - 1; y++) {
      if (!edgeStarted && src.at<uchar>(y, x) != 0) {
        lineStartY = y;
        edgeStarted = true;
        lineWidth = 1;
      } else if (edgeStarted) {
        lineWidth = y - lineStartY;

        if (lineWidth >= maxLineWidth) {
          edgeStarted = false;
        } else if (lineWidth >= minLineWidth && src.at<uchar>(y, x) != 0) {
          dst.at<uchar>(lineStartY + lineWidth / 2, x) = 255;
          edgeStarted = false;
        }
      }
    }
  }
}

void VisionLine::removeSmallEdgeComponents(cv::Mat &src, int minEdgeComponentArea) {
  cv::Mat labels, stats, centroids;
  int numLabels = cv::connectedComponentsWithStats(src, labels, stats, centroids, 8, CV_32S);

  for (int i = 1; i < numLabels; i++) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area <= minEdgeComponentArea) {
      cv::Mat mask = (labels == i);
      src.setTo(0, mask);
    }
  }
}

void VisionLine::Judgement_Line(const cv::Mat &src, const cv::Mat &edge, cv::Mat &leftLine, cv::Mat &rightLine, const int Y_value[6], const int W_value[6], bool is_parking) {
  cv::Mat srcHsv;
  cv::cvtColor(src, srcHsv, cv::COLOR_BGR2HSV);

  leftLine = cv::Mat::zeros(edge.size(), CV_8UC1);
  rightLine = cv::Mat::zeros(edge.size(), CV_8UC1);

  cv::Mat frameMaskYellow, frameMaskWhite;

  cv::inRange(srcHsv, cv::Scalar(Y_value[3], Y_value[4], Y_value[5]), cv::Scalar(Y_value[0], Y_value[1], Y_value[2]), frameMaskYellow);
  cv::inRange(srcHsv, cv::Scalar(W_value[3], W_value[4], W_value[5]), cv::Scalar(W_value[0], W_value[1], W_value[2]), frameMaskWhite);

  yellow_line = frameMaskYellow.clone();
  white_line = frameMaskWhite.clone();

  cv::Mat morphElem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(frameMaskYellow, frameMaskYellow, morphElem);
  cv::dilate(frameMaskWhite, frameMaskWhite, morphElem);

  constexpr int kernelSize = 1;
  for (int i = kernelSize; i < edge.rows - kernelSize; i++) {
    for (int j = kernelSize; j < edge.cols - kernelSize; j++) {
      if (edge.at<uchar>(i, j) != 0) {
        cv::Rect roi(j - kernelSize, i - kernelSize, 2 * kernelSize + 1, 2 * kernelSize + 1);
        cv::Mat yellowRoi = frameMaskYellow(roi);
        cv::Mat whiteRoi = frameMaskWhite(roi);

        int nYellow = cv::countNonZero(yellowRoi);
        int nWhite = cv::countNonZero(whiteRoi);

        if (Vision::parking_step[1]) {
          if (j < edge.cols / 2)
            leftLine.at<uchar>(i, j) = 255;
          else
            rightLine.at<uchar>(i, j) = 255;
        } else {
          if (nYellow >= 3)
            leftLine.at<uchar>(i, j) = 255;
          else if (nWhite >= 3)
            rightLine.at<uchar>(i, j) = 255;
        }
      }
    }
  }
}

void VisionLine::Extract_Line(const cv::Mat &src, const cv::Mat &leftLine, const cv::Mat &rightLine) {
  JYJ_RansacLine leftLineRansac(leftLine, true, false), rightLineRansac(rightLine, true, true);

  leftLineRansac.m_color = true;
  rightLineRansac.m_color = true;
  leftLineRansac.runRansac();
  rightLineRansac.runRansac();

  int64 right_line_start_X = rightLineRansac.getLineStart().x;
  int64 right_line_end_X = rightLineRansac.getLineEnd().x;
  int64 left_line_start_X = leftLineRansac.getLineStart().x;
  int64 left_line_end_X = leftLineRansac.getLineEnd().x;

  white_line_deg = rightLineRansac.getLineDegree();
  yellow_line_deg = leftLineRansac.getLineDegree();
  r_line_xpos = static_cast<int>((right_line_start_X + right_line_end_X) / 2);
  l_line_xpos = static_cast<int>((left_line_start_X + left_line_end_X) / 2);

  r_line_detect = rightLineRansac.m_done;
  l_line_detect = leftLineRansac.m_done;

  // if (!Vision::parking_ste[0]) {
  //   if (r_line_detect && l_line_detect && l_line_xpos > r_line_xpos) {
  //     r_line_detect = false;
  //   }
  // }

  if (r_line_detect) cv::line(src, rightLineRansac.getLineEnd(), rightLineRansac.getLineStart(), vision_colors::Red, 2, cv::LINE_AA);
  if (l_line_detect) cv::line(src, leftLineRansac.getLineEnd(), leftLineRansac.getLineStart(), vision_colors::Blue, 2, cv::LINE_AA);
}

void VisionLine::Extract_Line_Color(const cv::Mat &src, const cv::Mat &leftLine, const cv::Mat &rightLine) {
  VisionLabeling YellowLine(leftLine, 100);
  VisionLabeling WhiteLine(rightLine, 100);
  YellowLine.do_labeling();
  WhiteLine.do_labeling();

  y_line_detect = !YellowLine.blobs.empty();
  w_line_detect = !WhiteLine.blobs.empty();

  if (y_line_detect) {
    auto [min_it, max_it] = std::minmax_element(YellowLine.blobs.begin(), YellowLine.blobs.end(), [](const cv::Rect &a, const cv::Rect &b) { return a.x < b.x; });

    for (const auto &blob : YellowLine.blobs) {
      cv::rectangle(src, blob, vision_colors::Skyblue, 2);
    }
    cross_yellow_y = YellowLine.blobs[0].y + YellowLine.blobs[0].height;
  }

  if (w_line_detect) {
    auto [min_it, max_it] = std::minmax_element(WhiteLine.blobs.begin(), WhiteLine.blobs.end(), [](const cv::Rect &a, const cv::Rect &b) { return a.x < b.x; });

    for (const auto &blob : WhiteLine.blobs) {
      cv::rectangle(src, blob, vision_colors::Pink, 2);
    }

    Vision::white_line_blob_y = WhiteLine.blobs[0].y;
    Vision::white_line_blob_x = WhiteLine.blobs[0].x;
  }
  Vision::white_box_cnt = WhiteLine.blobs.size();
}

VisionLabeling::VisionLabeling(const cv::Mat &binary_img, int threshold) : img_binary(binary_img.clone()), pixel_threshold(threshold) {}

void VisionLabeling::do_labeling() {
  pixel_num = 0;
  cv::Mat labels, stats, centroids;
  num_of_labels = cv::connectedComponentsWithStats(img_binary, labels, stats, centroids, 8, CV_32S);

  for (int i = 1; i < num_of_labels; i++) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    int left = stats.at<int>(i, cv::CC_STAT_LEFT);
    int top = stats.at<int>(i, cv::CC_STAT_TOP);
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    cv::Mat roi = img_binary(cv::Rect(left, top, width, height));
    int blob_pixel_count = cv::countNonZero(roi);

    if (blob_pixel_count > pixel_threshold) {
      blobs.emplace_back(left, top, width, height);
      pixel_num = 0;
    }
  }
}

void VisionLabeling::draw_maximum_label_rect(cv::Mat &raw_img) {
  max_num = 0;
  max_pixel = 0;

  if (!blobs.empty()) {
    std::vector<int> num_of_pixel(blobs.size(), 0);

    for (size_t i = 0; i < blobs.size(); i++) {
      cv::Mat roi = img_binary(blobs[i]);
      num_of_pixel[i] = cv::countNonZero(roi);

      if (num_of_pixel[i] > max_pixel) {
        max_pixel = num_of_pixel[i];
        max_num = i;
      }
    }
  }
}