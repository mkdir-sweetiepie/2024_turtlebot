#include <QApplication>
#include <iostream>

#include "../include/robit_vision/main_window.hpp"

using namespace robit_vision;

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
