#include <QApplication>
#include <iostream>

#include "../include/robit_master/main_window.hpp"
using namespace robit_master;
int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
