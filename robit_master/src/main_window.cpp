#include "../include/robit_master/main_window.hpp"
namespace robit_master {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);
  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);
  qnode = new QNode();

  move(820, 0);  // ui위치
  QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(dataReceived()), this, SLOT(updateData()));
}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() { delete ui; }

void MainWindow::updateData() {
  // TUNNEL MSG
  ui->start2024->setText(QString::number(RobitDriving::start2024_));
  ui->label_imu->setText(QString::number(RobitDriving::direction_angle_));
  ui->label_rel->setText(QString::number(qnode->driving_.Vision_msg_.rel_angle));
  ui->rel_ratio->setText(QString::number(qnode->driving_.Vision_msg_.rel_angle_ratio));
  ui->label_cds->setText(QString::number(RobitDriving::cds_data_));

  // DRIVING MSG
  ui->label_linear_x->setText(QString::number(qnode->driving_.motor_value_.linear.x));
  ui->label_angular_z->setText(QString::number(qnode->driving_.motor_value_.angular.z));

  // LINE MSG
  if (qnode->driving_.Vision_msg_.l_line_info) {
    ui->label_left_detect->setText("Detect");
    ui->label_left_pixel->setText(QString::number(qnode->driving_.Vision_msg_.l_diff_pixel));
    ui->label_left_angle->setText(QString::number(qnode->driving_.Vision_msg_.l_angle));
  } else {
    ui->label_left_detect->setText("NO POINT");
    ui->label_left_pixel->setText("NO POINT");
  }
  if (qnode->driving_.Vision_msg_.r_line_info) {
    ui->label_right_detect->setText("Detect");
    ui->label_right_pixel->setText(QString::number(qnode->driving_.Vision_msg_.r_diff_pixel));
    ui->label_right_angle->setText(QString::number(qnode->driving_.Vision_msg_.r_angle));
  } else {
    ui->label_right_detect->setText("NO POINT");
    ui->label_right_pixel->setText("NO POINT");
  }

  // MISSION MSG
  ui->label_gatebar_detect->setText(qnode->driving_.Vision_msg_.gatebar_detect ? "Detect" : "NOPE");
  ui->label_parking_sign->setText(qnode->driving_.Vision_msg_.parking_detect ? "Detect" : "NOPE");
  ui->label_cross->setText(qnode->driving_.Vision_msg_.cross_detect ? "Detect" : "NOPE");
  ui->label_traffic_green->setText(qnode->driving_.Vision_msg_.traffic ? "Detect" : "NOPE");
}

void MainWindow::on_pushButton_clicked() {
  button_clicked = !button_clicked;
  ui->pushButton->setText(button_clicked ? "stop" : "start");
  qnode->driving_.master_msg_.traffic_done = button_clicked;
  RobitDriving::start2024_ = button_clicked;
}
}  // namespace robit_master