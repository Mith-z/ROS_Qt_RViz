#ifndef NODE_INFO_DRAWER_H
#define NODE_INFO_DRAWER_H

#include <QDebug>
#include <QFrame>
#include <QPainter>
#include <QPainterPath>
#include <QWidget>
#include <vector>

namespace qt_rviz_demo {

struct Point {
  int x;
  int y;
  Point(int x_point, int y_point) {
    x = 159 - x_point;
    y = 100 - y_point;
  }
};

class NodeInfoDrawer : public QFrame {
  Q_OBJECT

public:
  explicit NodeInfoDrawer(QWidget *parent = 0);
  ~NodeInfoDrawer();
  int X(int x); // trans coordinate
  int Y(float y);
  void add_data(float data);

  void change_queue();

private:
  void paintEvent(QPaintEvent *);

  int width = 160;
  int height = 101;
  int dist = 2;
  std::vector<Point> *data_queue;
  std::vector<Point> *cpu_queue;
  std::vector<Point> *mem_queue;
  int size = width / dist;
};
} // namespace qt_rviz_demo
#endif // NODE_INFO_DRAWER_H
