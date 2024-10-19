#ifndef robit_master_MAIN_WINDOW_H
#define robit_master_MAIN_WINDOW_H

#include <QMainWindow>

#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

namespace robit_master {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

 private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);

 public Q_SLOTS:
  void updateData(void);
  void on_pushButton_clicked();
};

}  // namespace robit_master
#endif  // robit_master_MAIN_WINDOW_H
