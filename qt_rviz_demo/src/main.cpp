#include "../include/qt_rviz_demo/main_window.hpp"

#include <QApplication>

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  ros::init(argc, argv, "qt_rviz_demo");
  ros::NodeHandle nh;
  qt_rviz_demo::MainWindow w;
  w.show();
  return a.exec();
}
