#include "../include/qt_rviz_demo/record_rosbag.h"
#include "ui_record_rosbag.h"

RecordROSBag::RecordROSBag(QWidget *parent)
    : QWidget(parent), ui(new Ui::RecordROSBag) {
  ui->setupUi(this);

  topicMultiComboBox = ui->topic_comboBox;

  // 创建一个QWidget作为浏览文件窗口
  QWidget *browseWindow;
  // 连接浏览按钮的点击事件到槽函数
  QObject::connect(ui->browse_Btn, &QPushButton::clicked, [=]() {
    QString selectedPath = QFileDialog::getExistingDirectory(
        browseWindow, "选择路径", QDir::homePath());

    if (!selectedPath.isEmpty()) {
      ui->dir_lineEdit->setText(selectedPath + "/record.bag");
    }
  });

  connect(ui->record_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnRecordBtnClickedSlot()));
}

RecordROSBag::~RecordROSBag() { delete ui; }

void RecordROSBag::recordRosbag(QList<std::string> &topics,
                                QMap<QString, QVariant> topicTypeMap,
                                const std::string &bag_filename,
                                double duration) {
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

void RecordROSBag::OnRecordBtnClickedSlot() {

  for (int i : topicMultiComboBox->currentIndex()) {
    topicsAndTypes.insert(topicMultiComboBox->itemText(i),
                          topicMultiComboBox->itemData(i));
  }

  QList<std::string> topicNames;
  recordRosbag(topicNames, topicsAndTypes,
               "/home/mith/catkin_qt/src/qt_rviz_demo/scripts/Bag/test3.bag",
               5.0);
}
