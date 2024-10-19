/**
 * @file /include/turtle_go/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef turtle_go_MAIN_WINDOW_H
#define turtle_go_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>

#include "QIcon"
#include "ui_mainwindow.h"
#include <chrono>
#include <thread>
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
 private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);

 public Q_SLOTS:
  void on_On_clicked();
  void on_Off_clicked();
};

#endif  // turtle_go_MAIN_WINDOW_H
