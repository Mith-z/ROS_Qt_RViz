#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qrviz.h"
#include <QAbstractItemModel>
#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

namespace qt_rviz_demo {

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  //窗口组件
  void initializeDockWidgets();
  void initRViz();

public slots:
  void GetRVizTreeModelSlot(QAbstractItemModel *model);

private:
  Ui::MainWindow *ui;

  qrviz *_qrviz;
  QAbstractItemModel *modelRvizDisplay;
};
#endif // MAINWINDOW_H

} // namespace qt_rviz_demo
