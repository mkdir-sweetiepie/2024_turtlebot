/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/turtle_go/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);
  move(0, 0);
  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);
}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_On_clicked() {
  system("gnome-terminal --geometry=80x18+0+300 -- bash -c 'ros2 launch yolov8_detection yolov8_detection_launch.py'");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // set_imu를 위치 0,0에 크기 800x300으로 실행
  system("gnome-terminal --geometry=80x18+0+300 -- bash -c 'ros2 run set_imu set_imu; exec bash'");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // robit_master를 위치 0,400에 크기 800x300으로 실행
  system("gnome-terminal --geometry=80x18+0+675 -- bash -c 'ros2 run robit_master robit_master; exec bash'");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // robit_vision을 위치 0,800에 크기 800x300으로 실행
  system("gnome-terminal --geometry=80x18+0+1050 -- bash -c 'ros2 run robit_vision robit_vision; exec bash'");
}

void MainWindow::on_Off_clicked() {
  system("pkill -f 'ros2 launch'");
  system("pkill -f set_imu");
  system("pkill -f robit_master");
  system("pkill -f robit_vision");
}
