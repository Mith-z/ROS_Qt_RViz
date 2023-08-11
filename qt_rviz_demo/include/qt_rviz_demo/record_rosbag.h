#ifndef RECORD_ROSBAG_H
#define RECORD_ROSBAG_H

#include <QObject>
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

class RecordPanel;

class RecordRosBag : public QObject {
  Q_OBJECT
public:
  explicit RecordRosBag();

signals:

public slots:
  void Record(QMap<QString, QVariant> topicTypeMap,
              const std::string &bag_filename, double duration);
  void Record(QMap<QString, QVariant> topicTypeMap,
              const std::string &bag_filename, RecordPanel *recordpanel);
};

#endif // RECORD_ROSBAG_H
