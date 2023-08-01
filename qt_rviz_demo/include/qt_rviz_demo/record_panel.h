#ifndef RECORD_PANEL_H
#define RECORD_PANEL_H

#include <QDoubleValidator>
#include <QtConcurrent/QtConcurrent>
#include <QtWidgets>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/macros.h>
#include <ros/master.h>
#include <ros/package.h>
#include <ros/subscriber.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "./UIPromoteClass/multiselect_combobox.h"
#include "record_rosbag.h"

namespace Ui {
class RecordPanel;
}

class RecordPanel : public QWidget {
  Q_OBJECT

public:
  explicit RecordPanel(QWidget *parent = nullptr);
  ~RecordPanel();

  MultiSelectComboBox *GetTopicComboBox() { return topicMultiComboBox; }

  void setMainWindowWidget(QWidget *window) { mainwindow = window; }

public slots:
  void OnRecordBtnClickedSlot();
  void OnCancelBtnClickedSlot();

private:
  Ui::RecordPanel *ui;
  MultiSelectComboBox *topicMultiComboBox;
  QMap<QString, QVariant> topicsAndTypes;
  int progressValue;

  QWidget *mainwindow;
  QThread *recordThread;

protected:
  void showEvent(QShowEvent *event) override;
  void closeEvent(QCloseEvent *event) override;
};

#endif // RECORD_PANEL_H
