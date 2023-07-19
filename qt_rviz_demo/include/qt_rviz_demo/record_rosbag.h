#ifndef RECORD_ROSBAG_H
#define RECORD_ROSBAG_H

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

namespace Ui {
class RecordROSBag;
}

class RecordROSBag : public QWidget {
  Q_OBJECT

public:
  explicit RecordROSBag(QWidget *parent = nullptr);
  ~RecordROSBag();

  MultiSelectComboBox *GetTopicComboBox() { return topicMultiComboBox; }

  void recordRosbag(QList<std::string> &topics,
                    QMap<QString, QVariant> topicTypeMap,
                    const std::string &bag_filename, double duration = 0);

public slots:
  void OnRecordBtnClickedSlot();

private:
  Ui::RecordROSBag *ui;
  MultiSelectComboBox *topicMultiComboBox;
  QMap<QString, QVariant> topicsAndTypes;
};

#endif // RECORD_ROSBAG_H
