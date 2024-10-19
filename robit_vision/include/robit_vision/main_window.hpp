#ifndef ROBIT_VISION_MAIN_WINDOW_HPP_
#define ROBIT_VISION_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <memory>
#include "qnode.hpp"
#include "ui_mainwindow.h"

namespace robit_vision {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  virtual ~MainWindow() = default;

 private:
  std::unique_ptr<Ui::MainWindowDesign> ui;
  std::unique_ptr<QNode> qnode;
  int click_flag;

  void closeEvent(QCloseEvent* event) override;
  void get_data();
  void update_label();
  void setupConnections();

 private Q_SLOTS:
  void updateFps(int fps);
  void updateTimer();
  void updateMissionData();
  void updateImage(const cv::Mat& image);
  void updatePerspectiveImg(const cv::Mat& perspective_img);
  void updatePerspectiveImg2(const cv::Mat& perspective_img2);
  void updateWhiteEdgeImg(const cv::Mat& white_line_img);
  void updateYellowEdgeImg(const cv::Mat& yellow_line_img);
  void updateTrafficImg(const cv::Mat& traffic_img);
  void updateBluesignImg(const cv::Mat& bluesign_img);
  void updateGatebarImg(const cv::Mat& gatebar_img);

  void on_Canny_value_valueChanged(int value);
  void on_Canny_ratio_valueChanged(int value);
  void on_upper_H_valueChanged(int value);
  void on_upper_S_valueChanged(int value);
  void on_upper_V_valueChanged(int value);
  void on_lower_H_valueChanged(int value);
  void on_lower_S_valueChanged(int value);
  void on_lower_V_valueChanged(int value);
  void on_Save_clicked();
  void on_mission_confirm_clicked();
  void on_Yellow_check_clicked();
  void on_Parking_check_clicked();
  void on_Gatebar_check_clicked();
  void on_traffic_check_clicked();
  void on_bluesign_clicked();
};

}  // namespace robit_vision

#endif  // ROBIT_VISION_MAIN_WINDOW_HPP_