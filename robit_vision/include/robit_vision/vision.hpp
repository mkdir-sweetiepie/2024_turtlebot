#ifndef VISION_H
#define VISION_H

#include <cv_bridge/cv_bridge.h>

#include <QObject>
#include <atomic>
#include <image_recognition_msgs/msg/bounding_box_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <robit_msgs/msg/master_msg.hpp>
#include <robit_msgs/msg/vision_msg.hpp>

#include "./robit_ransac.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vision_colors {
extern const cv::Scalar Red;
extern const cv::Scalar Blue;
extern const cv::Scalar Green;
extern const cv::Scalar Yellow;
extern const cv::Scalar CPL;
extern const cv::Scalar Pink;
extern const cv::Scalar Skyblue;
extern const cv::Scalar Mint;
}  // namespace vision_colors

enum class MissionType { CROSS, CONSTRUCT, PARKING, ZIGZAG, GATEBAR, TUNNEL };
enum class CrossDirection { NONE, LEFT, RIGHT };

class Vision : public QObject {
  Q_OBJECT

 public:
  Vision(QObject *parent = nullptr);
  ~Vision();

  void START(const cv::Mat &input_img);
  void Update_Message();
  void update_parameter(const std::shared_ptr<const robit_msgs::msg::MasterMsg> &master_data);
  void Retry_Button();

  static void Perspective_View(const cv::Mat &input_img, cv::Mat &output_img);
  static void Perspective_View2(const cv::Mat &input_img, cv::Mat &output_img);
  static void Change_to_Binary(const cv::Mat &input_img, cv::Mat &output_img, const int value[6], bool flag);

  static robit_msgs::msg::VisionMsg Vision_msg;
  static robit_msgs::msg::MasterMsg Master_msg;
  static image_recognition_msgs::msg::BoundingBoxMsgs BBX_msg;

  static constexpr int RAW_X = 320;
  static constexpr int RAW_Y = 240;
  static constexpr int PSD_SENSORS = 3;
  static constexpr int MISSION_COUNT = 6;

  static bool led[6];
  static int psd[PSD_SENSORS];
  static int count_fps, now_fps;
  static bool timer_ctrl;
  static int timer_cnt;
  static int traffic_value[6], parking_value[6], gatebar_value[6], bluesign_value[6];
  static bool rel_ctrl, rel_round;
  static int direction_angle, rel_zero_angle, rel_angle;
  static int start_imu[6];
  static double rel_angle_ratio;
  static int now_mission;
  static int retry;
  static int mission_sequence[MISSION_COUNT];

  // cross
  static bool cross_detect;
  static int cross_condition;
  static bool cross_step[5];

  bool cross_start;
  int direction_counter;

  std::atomic<bool> direction_flag;
  std::atomic<CrossDirection> cross_direction;

  // construct
  static bool construct_detect;
  static int construct_condition;
  static bool construct_start;
  static bool construct_step[6];
  bool construct_flag = false;

  // parking
  static bool parking_detect;
  static int parking_condition;
  static bool parking_step[9];
  static bool rotate;
  int cnt;

  static int just_before_tunnel_num;
  static int white_line_blob_y, white_line_blob_x;
  static int white_box_cnt;
  static unsigned long escape_standard;

 Q_SIGNALS:
  void perspective_callback(const cv::Mat &perspective_img);
  void perspective_callback2(const cv::Mat &perspective_img2);
  void white_line_callback(const cv::Mat &edge_img);
  void yellow_line_callback(const cv::Mat &edge_line_img);
  void mission_callback();
  void traffic_callback(const cv::Mat &traffic_img);
  void gatebar_callback(const cv::Mat &gatebar_img);

 private:
  cv::Mat Raw_image, Perspective_img, Perspective_img2;

  void finding_traffic_lights(const cv::Mat &raw_image, cv::Mat &draw_image);
  void Cross_Process();
  void Construct_Process();
  void Parking_Process();
  unsigned long parking_escape_bluesign(const cv::Mat &input_img);
  void Finding_Gatebar(cv::Mat &input_img);

  static int traffic_action;

  static bool zigzag_detect;
  static bool slow_zone;
  static bool gatebar_detect;
  static int gatebar_count;
  static bool gatebar_on;
  static bool gatebar_once;
  static bool bar_cnt_once;
  static bool bar_jjinmak;

  void setLed(bool _led1, bool _led2, bool _led3, bool _led4, bool _led5, bool _led6);

  enum Direction { LEFT, RIGHT };
  enum CrossCondition { APPROACHING, YELLOW_SIGN_PASSED, BLUE_SIGN_DETECTING, TURN_LEFT, TURN_RIGHT, DRIVE, CROSS_END_LEFT, CROSS_END_RIGHT, CROSS_END };
  enum ConstructCondition { SLOW, LEFT__, LEFT_C1, FRONT_C1, FRONT_C2, FRONT_C3, CONSTRUCT_END };
  enum ParkingCondition { GO_P1, GO_P2, LEFT_P1, GO_P3, DETECT, TURN2L, TURN2R, BACK, TURN3L, TURN3R, GO_P4, LEFT_P2, PARKING_END };
  enum MissionSequence { M_CROSS, M_CONSTRUCT, M_PARKING, M_ZIGZAG, M_GATEBAR, M_TUNNEL };
};

class VisionLine {
 public:
  VisionLine(const cv::Mat &perspective_img, bool is_parking, int num);

  static int r_line_xpos, l_line_xpos;
  static bool r_line_detect, l_line_detect, y_line_detect, w_line_detect;
  static double white_line_deg, yellow_line_deg;
  static int yellow_value[6], white_value[6];
  static int Canny_value[2];
  cv::Mat yellow_line, white_line, before_judg_yellow;
  static int cross_yellow_y;

 private:
  void MakeHorizontalOneLine(const cv::Mat &src, cv::Mat &dst, int minLineWidth = 10, int maxLineWidth = 45);
  void MakeVerticalOneLine(const cv::Mat &src, cv::Mat &dst, int minLineWidth = 10, int maxLineWidth = 45);
  void removeSmallEdgeComponents(cv::Mat &src, int minEdgeComponentArea = 15);
  void Judgement_Line(const cv::Mat &src, const cv::Mat &edge, cv::Mat &leftLine, cv::Mat &rightLine, const int Y_value[6], const int W_value[6], bool is_parking);
  void Extract_Line(const cv::Mat &src, const cv::Mat &leftLine, const cv::Mat &rightLine);
  void Extract_Line_Color(const cv::Mat &src, const cv::Mat &leftLine, const cv::Mat &rightLine);
};

class VisionLabeling {
 public:
  VisionLabeling(const cv::Mat &binary_img, int threshold);
  void do_labeling();
  void draw_maximum_label_rect(cv::Mat &raw_img);

  int num_of_labels, max_num, pixel_num, max_pixel;
  std::vector<cv::Rect> blobs;

 private:
  cv::Mat img_binary, img_labels, stats, centroids;
  int pixel_threshold;
};

#endif  // VISION_H