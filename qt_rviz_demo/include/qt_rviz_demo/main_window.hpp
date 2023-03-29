#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "adddisplay.h"
#include "image_view.h"
#include "qrviz.h"
#include "ratio_layouted_frame.h"

#include <QAbstractItemModel>
#include <QComboBox>
#include <QGraphicsPixmapItem>
#include <QMainWindow>
#include <QMessageBox>
#include <QResizeEvent>
#include <QSettings>

#include <boost/bind.hpp>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/macros.h>
#include <ros/package.h>
#include <rqt_gui_cpp/plugin.h>
#include <sensor_msgs/Image.h>

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
  void initUI();
  void initializeDockWidgets();
  void initRViz();

  void Connects();

public slots:
  void GetRVizTreeModelSlot(QAbstractItemModel *model);
  void OnAddDisplayBtnClickedSlot();
  void AddNewDisplaySlot(QTreeWidgetItem *newDisplay, QString name);
  // void CameraCheckBoxSlot();
  void Cam1TopicEditSlot(QString topic);

  void updateTopicList();

  void Cam1TopicChangedSlot(int index);
  void Cam2TopicChangedSlot(int index);
  void Cam3TopicChangedSlot(int index);
  void Cam4TopicChangedSlot(int index);

protected:
  QSet<QString> getTopics(const QSet<QString> &message_types,
                          const QSet<QString> &message_sub_types,
                          const QList<QString> &transports);
  void selectTopic(QComboBox *comboBox, const QString &topic);
  void OnCamTopicChanged(QComboBox *comboBox,
                         rqt_image_view::RatioLayoutedFrame *frame, int index,
                         int camSub);

  void callbackImage(const sensor_msgs::Image::ConstPtr &msg,
                     rqt_image_view::RatioLayoutedFrame *frame);

  cv::Mat conversion_mat_;
  int num_gridlines_;
  enum RotateState {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };
  RotateState rotate_state_;

private:
  Ui::MainWindow *ui;

  qrviz *_qrviz;
  QAbstractItemModel *modelRvizDisplay;
  AddDisplay *addDisplayPanel;

  QList<QComboBox *> camTopicComboBox;

  ros::NodeHandle nh_;

  QList<image_transport::Subscriber> camSubcribers;
  image_transport::Subscriber subscriber_cam1;
  image_transport::Subscriber subscriber_cam2;
  image_transport::Subscriber subscriber_cam3;
  image_transport::Subscriber subscriber_cam4;
};
#endif // MAINWINDOW_H

} // namespace qt_rviz_demo
