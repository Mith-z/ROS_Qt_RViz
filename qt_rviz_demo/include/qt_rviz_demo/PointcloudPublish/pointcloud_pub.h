#ifndef POINTCLOUD_PUB_H
#define POINTCLOUD_PUB_H

#include <iostream>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Pointcloud_Pub {
public:
  explicit Pointcloud_Pub(ros::NodeHandle *node);

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  std::string pointcloud_topic = "cloudpoint_topic";

  void initializePublishers();
  void initializeSubscribers();
  void callback(const sensor_msgs::PointCloud2ConstPtr &cloud);

public:
  void publish();
  int loadData(const char *file, void **data, unsigned int *length);
};

#endif // POINTCLOUD_PUB_H
