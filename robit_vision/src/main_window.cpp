#include "../include/robit_vision/main_window.hpp"

#include <QTimer>
#include <fstream>

namespace robit_vision {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(std::make_unique<Ui::MainWindowDesign>()), qnode(std::make_unique<QNode>()), click_flag(1) {
  ui->setupUi(this);

  get_data();
  update_label();
  setupConnections();

  QTimer* my_timer = new QTimer(this);
  my_timer->start(50);

  move(820, 330);
}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

void MainWindow::setupConnections() {
  connect(qnode.get(), &QNode::rosShutdown, this, &MainWindow::close);
  connect(qnode.get(), &QNode::fpsUpdated, this, &MainWindow::updateFps);
  connect(qnode.get(), &QNode::imageReceived, this, &MainWindow::updateImage);

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &MainWindow::updateTimer);
  timer->start(50);

  connect(&qnode->robit_vision, &Vision::mission_callback, this, &MainWindow::updateMissionData);
  connect(&qnode->robit_vision, &Vision::perspective_callback, this, &MainWindow::updatePerspectiveImg);
  connect(&qnode->robit_vision, &Vision::perspective_callback2, this, &MainWindow::updatePerspectiveImg2);
  connect(&qnode->robit_vision, &Vision::white_line_callback, this, &MainWindow::updateWhiteEdgeImg);
  connect(&qnode->robit_vision, &Vision::yellow_line_callback, this, &MainWindow::updateYellowEdgeImg);
  connect(&qnode->robit_vision, &Vision::traffic_callback, this, &MainWindow::updateTrafficImg);
  connect(&qnode->robit_vision, &Vision::bluesignDetected, this, &MainWindow::updateBluesignImg);
  connect(&qnode->robit_vision, &Vision::gatebar_callback, this, &MainWindow::updateGatebarImg);
}

void MainWindow::updateFps(int fps) { ui->fps_label->setText(QString::number(fps)); }

void MainWindow::updateTimer() {
  if (Vision::timer_ctrl)
    Vision::timer_cnt++;
  else
    Vision::timer_cnt = 0;

  ui->lcdNumber_2->display(QString::number(Vision::timer_cnt));
  ui->lcdNumber_psd1->display(QString::number(Vision::psd[0]));
  ui->lcdNumber_psd2->display(QString::number(Vision::psd[1]));
  ui->lcdNumber_psd3->display(QString::number(Vision::psd[2]));
}

void MainWindow::updateMissionData() {
  ui->now_mission->setText(QString::number(Vision::now_mission));
  ui->checkBox_Traffic->setChecked(Vision::Master_msg.traffic_done);
  ui->checkBox_Cross->setChecked(Vision::Master_msg.cross_done);
  ui->checkBox_Construct->setChecked(Vision::Master_msg.construct_done);
  ui->checkBox_Parking->setChecked(Vision::Master_msg.parking_done);
  ui->checkBox_Zigzag->setChecked(Vision::Master_msg.zigzag_done);
  ui->checkBox_Gatebar->setChecked(Vision::Master_msg.gatebar_done);
}

void MainWindow::updateImage(const cv::Mat& image) {
  if (!image.empty()) {
    cv::Mat resized, resized2;
    cv::resize(image, resized, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC);
    cv::resize(image, resized2, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);

    QImage qimage(resized.data, resized.cols, resized.rows, QImage::Format_RGB888);
    QImage qimage2(resized2.data, resized2.cols, resized2.rows, QImage::Format_RGB888);
    ui->label_raw_image->setPixmap(QPixmap::fromImage(qimage));
    ui->label_raw_image2->setPixmap(QPixmap::fromImage(qimage));
    ui->label_raw_image_2->setPixmap(QPixmap::fromImage(qimage2));
  }
}

void MainWindow::updatePerspectiveImg(const cv::Mat& perspective_img) {
  QImage qimage((const unsigned char*)(perspective_img.data), perspective_img.cols, perspective_img.rows, QImage::Format_RGB888);
  ui->label_perspective->setPixmap(QPixmap::fromImage(qimage));
}

void MainWindow::updatePerspectiveImg2(const cv::Mat& perspective_img2) {
  QImage qimage((const unsigned char*)(perspective_img2.data), perspective_img2.cols, perspective_img2.rows, QImage::Format_RGB888);
  ui->label_perspective2->setPixmap(QPixmap::fromImage(qimage));
}

void MainWindow::updateWhiteEdgeImg(const cv::Mat& white_line_img) {
  QImage qimage((const unsigned char*)(white_line_img.data), white_line_img.cols, white_line_img.rows, QImage::Format_Indexed8);
  ui->label_white_line->setPixmap(QPixmap::fromImage(qimage));
}

void MainWindow::updateYellowEdgeImg(const cv::Mat& yellow_line_img) {
  QImage qimage((const unsigned char*)(yellow_line_img.data), yellow_line_img.cols, yellow_line_img.rows, QImage::Format_Indexed8);
  ui->label_yellow_line->setPixmap(QPixmap::fromImage(qimage));
}

void MainWindow::updateTrafficImg(const cv::Mat& traffic_img) {
  cv::Mat rgb_img;
  cv::cvtColor(traffic_img, rgb_img, cv::COLOR_BGR2RGB);
  QImage qimage((const unsigned char*)(rgb_img.data), rgb_img.cols, rgb_img.rows, rgb_img.step, QImage::Format_RGB888);
  ui->label_traffic->setPixmap(QPixmap::fromImage(qimage));
}

void MainWindow::updateBluesignImg(const cv::Mat& bluesign_img) {
  cv::Mat resized;
  cv::resize(bluesign_img, resized, cv::Size(320, 240));
  QImage qimage((const unsigned char*)(resized.data), resized.cols, resized.rows, QImage::Format_Indexed8);
  ui->label_bluesign->setPixmap(QPixmap::fromImage(qimage));
}

void MainWindow::updateGatebarImg(const cv::Mat& gatebar_img) {
  QImage qimage((const unsigned char*)(gatebar_img.data), gatebar_img.cols, gatebar_img.rows, QImage::Format_Indexed8);
  ui->label_gatebar->setPixmap(QPixmap::fromImage(qimage));
}

//**********canny**********//
void MainWindow::on_Canny_value_valueChanged(int value) {
  VisionLine::Canny_value[0] = value;
  ui->label_Canny_value->setText(QString::number(VisionLine::Canny_value[0]));
}
void MainWindow::on_Canny_ratio_valueChanged(int value) {
  VisionLine::Canny_value[1] = value;
  ui->label_Canny_ratio->setText(QString::number(VisionLine::Canny_value[1]));
}

//**********HSV**********//
void MainWindow::on_upper_H_valueChanged(int value) {
  if (click_flag == 1)
    VisionLine::yellow_value[0] = value;
  else if (click_flag == 2)
    VisionLine::white_value[0] = value;
  else if (click_flag == 3)
    Vision::gatebar_value[0] = value;
  else if (click_flag == 4)
    Vision::traffic_value[0] = value;
  else if (click_flag == 5)
    Vision::bluesign_value[0] = value;
  ui->label_upper_H->setText(QString::number(value));
}
void MainWindow::on_upper_S_valueChanged(int value) {
  if (click_flag == 1)
    VisionLine::yellow_value[1] = value;
  else if (click_flag == 2)
    VisionLine::white_value[1] = value;
  else if (click_flag == 3)
    Vision::gatebar_value[1] = value;
  else if (click_flag == 4)
    Vision::traffic_value[1] = value;
  else if (click_flag == 5)
    Vision::bluesign_value[1] = value;
  ui->label_upper_S->setText(QString::number(value));
}
void MainWindow::on_upper_V_valueChanged(int value) {
  if (click_flag == 1)
    VisionLine::yellow_value[2] = value;
  else if (click_flag == 2)
    VisionLine::white_value[2] = value;
  else if (click_flag == 3)
    Vision::gatebar_value[2] = value;
  else if (click_flag == 4)
    Vision::traffic_value[2] = value;
  else if (click_flag == 5)
    Vision::bluesign_value[2] = value;
  ui->label_upper_V->setText(QString::number(value));
}
void MainWindow::on_lower_H_valueChanged(int value) {
  if (click_flag == 1)
    VisionLine::yellow_value[3] = value;
  else if (click_flag == 2)
    VisionLine::white_value[3] = value;
  else if (click_flag == 3)
    Vision::gatebar_value[3] = value;
  else if (click_flag == 4)
    Vision::traffic_value[3] = value;
  else if (click_flag == 5)
    Vision::bluesign_value[3] = value;
  ui->label_lower_H->setText(QString::number(value));
}
void MainWindow::on_lower_S_valueChanged(int value) {
  if (click_flag == 1)
    VisionLine::yellow_value[4] = value;
  else if (click_flag == 2)
    VisionLine::white_value[4] = value;
  else if (click_flag == 3)
    Vision::gatebar_value[4] = value;
  else if (click_flag == 4)
    Vision::traffic_value[4] = value;
  else if (click_flag == 5)
    Vision::bluesign_value[4] = value;
  ui->label_lower_S->setText(QString::number(value));
}
void MainWindow::on_lower_V_valueChanged(int value) {
  if (click_flag == 1)
    VisionLine::yellow_value[5] = value;
  else if (click_flag == 2)
    VisionLine::white_value[5] = value;
  else if (click_flag == 3)
    Vision::gatebar_value[5] = value;
  else if (click_flag == 4)
    Vision::traffic_value[5] = value;
  else if (click_flag == 5)
    Vision::bluesign_value[5] = value;
  ui->label_lower_V->setText(QString::number(value));
}

// 각 항목별 HSV값을 txt로 저장함.
void MainWindow::on_Save_clicked() {
  std::ofstream save_parameter(qnode->packagePath + "/parameter_data.txt");

  if (save_parameter.is_open()) {
    for (int i = 0; i < 6; i++) save_parameter << VisionLine::yellow_value[i] << std::endl;
    for (int i = 0; i < 2; i++) save_parameter << VisionLine::Canny_value[i] << std::endl;
    for (int i = 0; i < 6; i++) save_parameter << VisionLine::white_value[i] << std::endl;
    for (int i = 0; i < 6; i++) save_parameter << Vision::gatebar_value[i] << std::endl;
    for (int i = 0; i < 6; i++) save_parameter << Vision::traffic_value[i] << std::endl;
    for (int i = 0; i < 6; i++) save_parameter << Vision::bluesign_value[i] << std::endl;
  }
  save_parameter.close();
}

// 미션의 순서와 기준 IMU를 지정 후 txt로 저장함.
void MainWindow::on_mission_confirm_clicked() {
  for (int i = 0; i < 6; i++) {
    QTableWidgetItem* theItem = ui->mission_set->item(i, 0);
    QString sequence = theItem->text();
    Vision::mission_sequence[i] = sequence.toInt();
  }

  for (int i = 0; i < 6; i++) {
    QTableWidgetItem* theItem = ui->mission_set->item(i, 1);
    QString start_imu = theItem->text();
    Vision::start_imu[i] = start_imu.toInt();
  }
  Vision::just_before_tunnel_num = Vision::mission_sequence[5] - 1;

  std::ofstream save_mission(qnode->packagePath + "/mission_data.txt");

  if (save_mission.is_open()) {
    for (int i = 0; i < 6; i++) save_mission << Vision::mission_sequence[i] << std::endl;
    for (int i = 0; i < 6; i++) save_mission << Vision::start_imu[i] << std::endl;
  }
  save_mission.close();
}

// 저장한 각 항목의 HSV 값을 받아옴.
void MainWindow::get_data() {
  std::ifstream get_parameter(qnode->packagePath + "/parameter_data.txt");

  if (get_parameter.is_open()) {
    for (int i = 0; i < 6; i++) get_parameter >> VisionLine::yellow_value[i];
    for (int i = 0; i < 2; i++) get_parameter >> VisionLine::Canny_value[i];
    for (int i = 0; i < 6; i++) get_parameter >> VisionLine::white_value[i];
    for (int i = 0; i < 6; i++) get_parameter >> Vision::gatebar_value[i];
    for (int i = 0; i < 6; i++) get_parameter >> Vision::traffic_value[i];
    for (int i = 0; i < 6; i++) get_parameter >> Vision::bluesign_value[i];
  }
  get_parameter.close();
}

void MainWindow::update_label() {
  int paste_array[6] = {
      0,
  };
  if (click_flag == 1) {
    for (int i = 0; i < 6; i++) paste_array[i] = VisionLine::yellow_value[i];
  } else if (click_flag == 2) {
    for (int i = 0; i < 6; i++) paste_array[i] = VisionLine::white_value[i];
  } else if (click_flag == 3) {
    for (int i = 0; i < 6; i++) paste_array[i] = Vision::gatebar_value[i];
  } else if (click_flag == 4) {
    for (int i = 0; i < 6; i++) paste_array[i] = Vision::traffic_value[i];
  } else if (click_flag == 5) {
    for (int i = 0; i < 6; i++) paste_array[i] = Vision::bluesign_value[i];
  }

  for (int i = 0; i < 6; i++) {
    QString mission = QString::number(Vision::mission_sequence[i]);
    ui->mission_set->setItem(i, 0, new QTableWidgetItem(mission));

    QString imu = QString::number(Vision::start_imu[i]);
    ui->mission_set->setItem(i, 1, new QTableWidgetItem(imu));
  }

  ui->label_upper_H->setText(QString::number(paste_array[0]));
  ui->upper_H->setValue(paste_array[0]);
  ui->label_upper_S->setText(QString::number(paste_array[1]));
  ui->upper_S->setValue(paste_array[1]);
  ui->label_upper_V->setText(QString::number(paste_array[2]));
  ui->upper_V->setValue(paste_array[2]);
  ui->label_lower_H->setText(QString::number(paste_array[3]));
  ui->lower_H->setValue(paste_array[3]);
  ui->label_lower_S->setText(QString::number(paste_array[4]));
  ui->lower_S->setValue(paste_array[4]);
  ui->label_lower_V->setText(QString::number(paste_array[5]));
  ui->lower_V->setValue(paste_array[5]);
  ui->label_Canny_value->setText(QString::number(VisionLine::Canny_value[0]));
  ui->Canny_value->setValue(VisionLine::Canny_value[0]);
  ui->label_Canny_ratio->setText(QString::number(VisionLine::Canny_value[1]));
  ui->Canny_ratio->setValue(VisionLine::Canny_value[1]);

  ui->label_12->setText(QString::number(Vision::rel_angle_ratio));
}

void MainWindow::on_Yellow_check_clicked() {
  ui->Parking_check->setChecked(false);
  ui->Gatebar_check->setChecked(false);
  ui->traffic_check->setChecked(false);
  ui->bluesign->setChecked(false);

  click_flag = 1;
  update_label();
}

void MainWindow::on_Parking_check_clicked() {
  ui->Yellow_check->setChecked(false);
  ui->Gatebar_check->setChecked(false);
  ui->traffic_check->setChecked(false);
  ui->bluesign->setChecked(false);

  click_flag = 2;
  update_label();
}

void MainWindow::on_Gatebar_check_clicked() {
  ui->Yellow_check->setChecked(false);
  ui->Parking_check->setChecked(false);
  ui->traffic_check->setChecked(false);
  ui->bluesign->setChecked(false);

  click_flag = 3;
  update_label();
}

void MainWindow::on_traffic_check_clicked() {
  ui->Gatebar_check->setChecked(false);
  ui->Yellow_check->setChecked(false);
  ui->Parking_check->setChecked(false);
  ui->bluesign->setChecked(false);

  click_flag = 4;
  update_label();
}

void MainWindow::on_bluesign_clicked() {
  ui->Gatebar_check->setChecked(false);
  ui->Yellow_check->setChecked(false);
  ui->Parking_check->setChecked(false);
  ui->traffic_check->setChecked(false);

  click_flag = 5;
  update_label();
}

}  // namespace robit_vision