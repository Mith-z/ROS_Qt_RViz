//#include "../include/qt_rviz_demo/record_rosbag.h"
#include "../include/qt_rviz_demo/record_panel.h"

RecordRosBag::RecordRosBag() {}

void RecordRosBag::Record(QMap<QString, QVariant> topicTypeMap,
                          const std::string &bag_filename, double duration) {
  QList<std::string> topics;
  // 创建ROS节点
  ros::NodeHandle nh;

  // 创建ROSbag对象
  rosbag::Bag bag;

  bag.open(bag_filename, rosbag::bagmode::Write);

  // 创建ROS话题订阅器
  std::vector<ros::Subscriber> subs;

  QMap<QString, QVariant>::iterator it;
  for (it = topicTypeMap.begin(); it != topicTypeMap.end(); ++it) {
    topics.push_back(it.key().toStdString());
  }

  for (const auto &topic : topics) {
    if (topicTypeMap.value(QString::fromStdString(topic)) ==
        "sensor_msgs/Image") {
      qDebug() << "write image";
      subs.push_back(nh.subscribe<sensor_msgs::Image>(
          topic, 1, [&](const sensor_msgs::Image::ConstPtr &msg) {
            bag.write(topic, ros::Time::now(), *msg);
          }));
    } else if (topicTypeMap.value(QString::fromStdString(topic)) ==
               "sensor_msgs/PointCloud2") {
      qDebug() << "write pc2";
      subs.push_back(nh.subscribe<sensor_msgs::PointCloud2>(
          topic, 1, [&](const sensor_msgs::PointCloud2::ConstPtr &msg) {
            bag.write(topic, ros::Time::now(), *msg);
          }));
    } else {
      ROS_WARN_STREAM("Unknown topic: " << topic);
    }
  }

  // 持续录制数据
  if (duration <= 0) {
    ROS_INFO_STREAM("Start recording indefinitely...");
    ros::spin();
  }
  // 持续录制一段时间后停止
  else {
    ROS_INFO_STREAM("Start recording for " << duration << " seconds...");

    ros::Time start_time = ros::Time::now();
    while ((ros::Time::now() - start_time).toSec() < duration) {
      ros::spinOnce();
    }
  }

  // 停止订阅器并关闭ROSbag文件
  for (auto sub : subs) {
    sub.shutdown();
  }
  bag.close();

  ROS_INFO_STREAM("Recording finished!");
}

void RecordRosBag::Record(QMap<QString, QVariant> topicTypeMap,
                          const std::string &bag_filename,
                          RecordPanel *recordpanel) {
  QList<std::string> topics;
  // 创建ROS节点
  ros::NodeHandle nh;

  // 创建ROSbag对象
  rosbag::Bag bag;

  bag.open(bag_filename, rosbag::bagmode::Write);

  // 创建ROS话题订阅器
  std::vector<ros::Subscriber> subs;

  QMap<QString, QVariant>::iterator it;
  for (it = topicTypeMap.begin(); it != topicTypeMap.end(); ++it) {
    topics.push_back(it.key().toStdString());
  }

  for (const auto &topic : topics) {
    if (topicTypeMap.value(QString::fromStdString(topic)) ==
        "sensor_msgs/Image") {
      qDebug() << "write image";
      subs.push_back(nh.subscribe<sensor_msgs::Image>(
          topic, 1, [&](const sensor_msgs::Image::ConstPtr &msg) {
            bag.write(topic, ros::Time::now(), *msg);
          }));
    } else if (topicTypeMap.value(QString::fromStdString(topic)) ==
               "sensor_msgs/PointCloud2") {
      qDebug() << "write pc2";
      subs.push_back(nh.subscribe<sensor_msgs::PointCloud2>(
          topic, 1, [&](const sensor_msgs::PointCloud2::ConstPtr &msg) {
            bag.write(topic, ros::Time::now(), *msg);
          }));
    } else {
      ROS_WARN_STREAM("Unknown topic: " << topic);
    }
  }

  //持续录制
  while (!recordpanel->GetStopRecord()) {
    ros::spinOnce();
  }

  // 停止订阅器并关闭ROSbag文件
  for (auto sub : subs) {
    sub.shutdown();
  }
  bag.close();

  ROS_INFO_STREAM("Recording finished!");
}
