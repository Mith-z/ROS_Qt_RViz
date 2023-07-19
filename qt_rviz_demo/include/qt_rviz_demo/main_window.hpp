#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "PointcloudPublish/pointcloud_pub.h"
#include "UIPromoteClass/cam_mdisubwindow.h"
#include "UIPromoteClass/ratio_layouted_frame.h"
#include "adddisplay.h"
#include "nodeinfo.h"
#include "qrviz.h"
#include "record_rosbag.h"

#include <QtWidgets>

#include <Python.h>
#include <boost/bind.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/macros.h>
#include <ros/master.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <thread>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

namespace qt_rviz_demo {

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr, int argc = 0, char *argv[] = {});
  ~MainWindow();

  Ui::MainWindow *ui;
  //窗口组件
  void initUI();
  void initializeDockWidgets();
  void initRViz();
  void Connects();

  //  static void
  //  FrontPoint_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

public slots:
  void GetRVizTreeModelSlot(QAbstractItemModel *model);
  void OnAddDisplayBtnClickedSlot();
  void AddNewDisplaySlot(QTreeWidgetItem *newDisplay, QString name);
  void OnRemoveDisplayBtnClickedSlot();

  // void CameraCheckBoxSlot();

  void OnAddCamBtnClickedSlot();
  void OnResizeCamBtnClickedSlot();
  void OnCamComboBoxClickedSlot(CamMdiSubWindow *subwindow);

  void DataTopicChangedSlot(QString topicName);
  void UpdatePC2Data();
  void UpdateDataTreeView();

  //工具栏
  void PauseActionClickedSlot();
  void RecordActionClickedSlot();

  //模型
  void DeteceBtnSlot();
  void PublishBtnSlot();

  bool eventFilter(QObject *watched, QEvent *event);

protected:
  // camera
  QSet<QString> GetTopics(const QSet<QString> &message_types,
                          const QSet<QString> &message_sub_types,
                          const QList<QString> &transports);
  void SelectTopic(QComboBox *comboBox, const QString &topic);
  void CamTopicChanged(CamMdiSubWindow *camWindow, int index);

  void callbackImage(const sensor_msgs::Image::ConstPtr &msg,
                     rqt_image_view::RatioLayoutedFrame *frame);

  void rosbagCallback(const std_msgs::String::ConstPtr &msg);
  void recordRosbag(const QList<std::string> &topics,
                    const std::string &bag_filename, double duration = 0);

  // data treeview
  template <typename T> QString typeToString(T type);
  void InitPC2Model();
  void AddTreeViewRow(QStandardItem *parentItem, QString name, QString data,
                      QList<QStandardItem *> items);
  void AddTreeViewRow(QStandardItemModel *parentModel, QString name,
                      QString data, QList<QStandardItem *> items);
  QMap<QString, QString> GetAllTopicsAndTypes();
  void OnDataTopicChanged(QString topicName);

  void pythonTest();
  void static run_python_code(PyObject *callback);

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
  ros::NodeHandle nh_;
  ros::NodeHandle *nh;
  nodeinfo *node_;
  Pointcloud_Pub *pub;
  qrviz *_qrviz;

  QTimer *dataTimer;
  QThread *dataTimerThread;
  // ui
  QAbstractItemModel *modelRvizDisplay;
  AddDisplay *addDisplayPanel;
  RecordROSBag *recordBagPanel;
  QList<QAction *> toolBarActions;
  // camera
  QList<QWidget *> camWidgets;
  QMdiArea *mdiArea;
  QList<QComboBox *> camTopicComboBox;
  QList<CamMdiSubWindow *> camSubWindows;
  // data
  QStandardItemModel *emptyModel;
  QStandardItemModel *pc2Model;

  rosbag::Bag bag;
};
#endif // MAINWINDOW_H

} // namespace qt_rviz_demo
