#include "../include/qt_rviz_demo/main_window.hpp"
#include <QApplication>
#include <QTimer>
#include <json/json.h>

size_t WriteCallback(void *contents, size_t size, size_t nmemb,
                     std::string *output) {
  size_t total_size = size * nmemb;
  output->append(static_cast<char *>(contents), total_size);
  return total_size;
}

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  ros::init(argc, argv, "qt_rviz_demo");
  ros::NodeHandle nh;

  qt_rviz_demo::MainWindow w(nullptr, argc, argv);
  w.setObjectName("mymainwindow");
  w.show();

  return a.exec();
}
