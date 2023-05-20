#include "./include/qt_rviz_demo/PointcloudPublish/pointcloud_pub.h"

Pointcloud_Pub::Pointcloud_Pub(ros::NodeHandle *node) : nh(*node) {
  initializePublishers();
  initializeSubscribers();
}

void Pointcloud_Pub::initializePublishers() {
  pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
}

void Pointcloud_Pub::initializeSubscribers() {
  sub = nh.subscribe(pointcloud_topic, 10, &Pointcloud_Pub::callback, this);
}

void Pointcloud_Pub::callback(const sensor_msgs::PointCloud2ConstPtr &cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *temp_cloud);
  // or
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

  // do stuff
  ROS_INFO("received %ld points", temp_cloud->points.size());
}

void Pointcloud_Pub::publish() {
  sensor_msgs::PointCloud2 msg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::string dataFile =
      "/home/mith/catkin_qt/src/qt_rviz_demo/scripts/pointcloud/000000.bin";
  unsigned int length = 0;
  void *data = NULL;
  std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());
  loadData(dataFile.data(), &data, &length);
  buffer.reset((char *)data);

  float *points = (float *)buffer.get();
  size_t points_size = length / sizeof(float) / 4;

  std::cout << points[2] << std::endl << points_size << std::endl;

  //  for (size_t i = 0; i < points_size; i++) {
  //    pcl::PointXYZ p(1.0 * i, 2.0 * i, 3.0 * i);
  //    cloud->push_back(p);
  //  }
  pcl::PointXYZ p(points[0], points[1], points[2]);
  cloud->push_back(p);

  pcl::toROSMsg(*cloud, msg);
  // or
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*cloud, pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, msg);

  // publish
  pub.publish(msg);
  ROS_INFO("published.");
}

int Pointcloud_Pub::loadData(const char *file, void **data,
                             unsigned int *length) {
  std::fstream dataFile(file, std::ifstream::in);

  if (!dataFile.is_open()) {
    std::cout << "Can't open files: " << file << std::endl;
    return -1;
  }

  // get length of file:
  unsigned int len = 0;
  dataFile.seekg(0, dataFile.end);
  len = dataFile.tellg();
  dataFile.seekg(0, dataFile.beg);

  // allocate memory:
  char *buffer = new char[len];
  if (buffer == NULL) {
    std::cout << "Can't malloc buffer." << std::endl;
    dataFile.close();
    exit(-1);
  }

  // read data as a block:
  dataFile.read(buffer, len);
  dataFile.close();

  *data = (void *)buffer;
  *length = len;
  return 0;
}
