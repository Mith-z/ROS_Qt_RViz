#include "../include/qt_rviz_demo/main_window.hpp"
#include "../include/qt_rviz_demo/qrviz.h"
#include "ui_main_window.h"
#include <QDebug>
std::string FrontPcd_path;

namespace qt_rviz_demo {

MainWindow::MainWindow(QWidget *parent, int argc, char *argv[])
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  node_ = new nodeinfo(ui, argc, argv);
  initUI();
  initRViz();
  Connects();

  //  std::string front_lidar_topic;

  //  nh_.param("front_lidar_topic", front_lidar_topic,
  //            std::string("/rslidar_points"));
  //  nh_.param(
  //      "FrontPcd_path", FrontPcd_path,
  //      std::string("/home/mith/catkin_qt/src/qt_rviz_demo/scripts/Lidar/"));
  //  ros::Subscriber sub_cloud1 =
  //      nh_.subscribe(front_lidar_topic, 100, FrontPoint_callback);
  //  ros::MultiThreadedSpinner spinner(1);
  //  spinner.spin();
}

void MainWindow::FrontPoint_callback(
    const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  ros::Time time = cloud_msg->header.stamp;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  pcl::fromROSMsg(*cloud_msg, tmp);
  double tt = time.toSec();

  // save to pcd file
  pcl::io::savePCDFileBinary(FrontPcd_path + std::to_string(tt) + ".pcd", tmp);
  pcl::io::savePCDFileASCII(FrontPcd_path + std::to_string(tt) + ".pcd", tmp);
  pcl::io::savePLYFile(FrontPcd_path + std::to_string(tt) + ".ply", tmp);
  // save to bin file
  std::ofstream out;
  std::string save_filename = FrontPcd_path + std::to_string(tt) + ".bin";
  out.open(save_filename, std::ios::out | std::ios::binary);
  std::cout << save_filename << " saved" << std::endl;
  int cloudSize = tmp.points.size();
  for (int i = 0; i < cloudSize; ++i) {
    float point_x = tmp.points[i].x;
    float point_y = tmp.points[i].y;
    float point_z = tmp.points[i].z;
    out.write(reinterpret_cast<const char *>(&point_x), sizeof(float));
    out.write(reinterpret_cast<const char *>(&point_y), sizeof(float));
    out.write(reinterpret_cast<const char *>(&point_z), sizeof(float));
  }
  out.close();
}

void MainWindow::pythonTest() {
  Py_Initialize();
  PyRun_SimpleString("import sys");
  PyRun_SimpleString(
      "sys.path.append('/home/mith/catkin_qt/src/qt_rviz_demo/scripts')");

  PyObject *pModule = PyImport_ImportModule("pcl_publish");
  if (pModule == NULL) {
    PyErr_Print();
    qDebug("cannot find module");
  }

  PyObject *pFunc = PyObject_GetAttrString(pModule, "func");
  if (pFunc == NULL) {
    PyErr_Print();
    qDebug("cannot find function");
  }

  PyObject *pRet = PyObject_CallObject(pFunc, NULL);

  //  char *result;
  //  PyArg_Parse(pRet, "s", &result);
  //  qDebug() << QString(QLatin1String(result));
  Py_Finalize();
}

MainWindow::~MainWindow() {
  delete ui;
  if (dataTimer->isActive()) {
    dataTimerThread->terminate();
  }
  if (dataTimerThread->isRunning()) {
    dataTimerThread->terminate();
  }
  dataTimerThread->deleteLater();
  dataTimerThread->wait();
  delete dataTimerThread;
  delete dataTimer;
}

/**
 * @brief 初始化主窗口UI
 */
void MainWindow::initUI() {
  initializeDockWidgets();
  addDisplayPanel = new AddDisplay();

  // toolbar
  toolBarActions = ui->toolBar->actions();
  toolBarActions.at(0)->setChecked(true);
  QActionGroup *group = new QActionGroup(this);
  for (int i = 0; i < toolBarActions.size(); i++) {
    group->addAction(toolBarActions.at(i));
  }

  // camera
  camTopicComboBox.append(ui->cam1_topic_comboBox);
  camTopicComboBox.append(ui->cam2_topic_comboBox);
  camTopicComboBox.append(ui->cam3_topic_comboBox);
  camTopicComboBox.append(ui->cam4_topic_comboBox);

  ui->cam_update_Btn->setIcon(QIcon("://images/refreash.png"));

  camSubcribers.append(subscriber_cam1);
  camSubcribers.append(subscriber_cam2);
  camSubcribers.append(subscriber_cam3);
  camSubcribers.append(subscriber_cam4);
  // data
  emptyModel = new QStandardItemModel();
  dataTimer = new QTimer();
  dataTimerThread = new QThread(this);
  InitPC2Model();
  ui->data_topic_comboBox->installEventFilter(this);

  ui->displayTabWidget->setCurrentIndex(0);
}

/**
 * @brief 初始化RViz显示
 */
void MainWindow::initRViz() {
  _qrviz = new qrviz(ui->rvizLayout);
  connect(_qrviz, &qrviz::DisplayTreeModelSignal, this,
          &MainWindow::GetRVizTreeModelSlot);
  _qrviz->GetRVizDisplayTreeModel();

  QMap<QString, QVariant> value;
  value.insert("Line Style", "Billboards");
  value.insert("Color", QColor(160, 160, 160));
  value.insert("Plane Cell Count", 10);
  _qrviz->InitDisplayTreeModel("rviz/Grid", "Grid", true, value);

  QMap<QString, QVariant> value1;
  value1.insert("Topic", "/kitti_point_cloud_origin");
  _qrviz->InitDisplayTreeModel("rviz/PointCloud2", "pc2", true, value1);
}

/**
 * @brief slot， 在ui上显示rviz的各个display type
 * @param model
 */
void MainWindow::GetRVizTreeModelSlot(QAbstractItemModel *model) {
  modelRvizDisplay = model;
  ui->typeTreeView->setModel(model);
}

/**
 * @brief 初始化DockWidgets
 */
void MainWindow::initializeDockWidgets() {
  // DockWidgets
  QDockWidget *typeDock = findChild<QDockWidget *>("typeDock");
  QDockWidget *infoDock = findChild<QDockWidget *>("infoDock");
  QDockWidget *dataDock = findChild<QDockWidget *>("dataDock");
  QDockWidget *cameraDock = findChild<QDockWidget *>("cameraDock");
  // DockWidgets初始吸附位置
  this->addDockWidget(Qt::RightDockWidgetArea, dataDock);
  this->addDockWidget(Qt::BottomDockWidgetArea, cameraDock);

  ui->typeDock->setStyleSheet(
      QString::fromUtf8("#typeDock{border:1px solid grey}"));

  //设置初始大小
  this->resizeDocks({typeDock, infoDock, dataDock}, {288, 288, 280},
                    Qt::Horizontal);
  this->resizeDocks({typeDock, infoDock, cameraDock}, {300, 200, 175},
                    Qt::Vertical);

  //合并
  this->tabifyDockWidget(typeDock, infoDock);
  typeDock->raise();
}

/**
 * @brief 链接各个Signal和Slot
 */
void MainWindow::Connects() {

  connect(ui->add_display_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnAddDisplayBtnClickedSlot()));
  connect(ui->remove_display_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnRemoveDisplayBtnClickedSlot()));

  connect(_qrviz, SIGNAL(AddNewDisplaySignal(QString)), addDisplayPanel,
          SLOT(AddNewDisplaySlot(QString)));
  connect(addDisplayPanel,
          SIGNAL(ChooseDisplaySignal(QTreeWidgetItem *, QString)), this,
          SLOT(AddNewDisplaySlot(QTreeWidgetItem *, QString)));

  // toolbar
  connect(ui->pause_action, SIGNAL(triggered(bool)), this,
          SLOT(PauseActionClickedSlot()));

  // camera
  connect(ui->cam_update_Btn, SIGNAL(clicked(bool)), this,
          SLOT(updateTopicList()));
  connect(ui->cam1_topic_comboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(Cam1TopicChangedSlot(int)));
  connect(ui->cam2_topic_comboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(Cam2TopicChangedSlot(int)));
  connect(ui->cam3_topic_comboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(Cam3TopicChangedSlot(int)));
  connect(ui->cam4_topic_comboBox, SIGNAL(currentIndexChanged(int)), this,
          SLOT(Cam4TopicChangedSlot(int)));

  // data treeview
  connect(ui->data_topic_comboBox, SIGNAL(currentIndexChanged(QString)), this,
          SLOT(DataTopicChangedSlot(QString)));

  //模型
  connect(ui->detect_Btn, SIGNAL(clicked(bool)), this, SLOT(DeteceBtnSlot()));
  connect(ui->publish_Btn, SIGNAL(clicked(bool)), this, SLOT(PublishBtnSlot()));
}

/**
 * @brief 添加Display按钮
 */
void MainWindow::OnAddDisplayBtnClickedSlot() { addDisplayPanel->show(); }

/**
 * @brief 移除Display按钮
 */
void MainWindow::OnRemoveDisplayBtnClickedSlot() {
  QModelIndex current = ui->typeTreeView->currentIndex();
  if (current.isValid()) {
    current = current.sibling(current.row(), 0);
    QString name =
        ui->typeTreeView->model()->itemData(current).values()[0].toString();
    qDebug() << name;

    _qrviz->RemoveDisplay(name);
    if (addDisplayPanel->allDisplayNames.contains(name))
      addDisplayPanel->allDisplayNames.removeOne(name);
  }
}

/**
 * @brief slot 添加display时调用
 * @param newDisplay QTreeWidgetItem对象
 * @param name 新建Display名称
 */
void MainWindow::AddNewDisplaySlot(QTreeWidgetItem *newDisplay, QString name) {
  QMap<QString, QVariant> Value;
  Value.clear();
  qDebug() << "添加Disaplay  " + name;
  _qrviz->InitDisplayTreeModel("rviz/" + newDisplay->text(0), name, true,
                               Value);
}

/**
 * @brief slot 选择camera topic时调用
 * @param camera的combobox指数
 */
void MainWindow::Cam1TopicChangedSlot(int index) {
  OnCamTopicChanged(ui->cam1_topic_comboBox, ui->cam1_frame, index, 1);
}
void MainWindow::Cam2TopicChangedSlot(int index) {
  OnCamTopicChanged(ui->cam2_topic_comboBox, ui->cam2_frame, index, 2);
}
void MainWindow::Cam3TopicChangedSlot(int index) {
  OnCamTopicChanged(ui->cam3_topic_comboBox, ui->cam3_frame, index, 3);
}
void MainWindow::Cam4TopicChangedSlot(int index) {
  OnCamTopicChanged(ui->cam4_topic_comboBox, ui->cam4_frame, index, 4);
}

void MainWindow::OnInfoUpdateBtnClickedSlot() {
  system("rosrun rqt_top rqt_top");
}

//工具栏slot
void MainWindow::PauseActionClickedSlot() {
  QString fixedFrame = _qrviz->getVisualizationManager()->getFixedFrame();
  if (ui->pause_action->text() == "暂停") {
    ui->pause_action->setText("播放");

    _qrviz->getVisualizationManager()->getFrameManager()->setFixedFrame("");

  } else {
    ui->pause_action->setText("暂停");

    _qrviz->getVisualizationManager()->getFrameManager()->setFixedFrame(
        fixedFrame.toStdString());

    rviz::DisplayGroup *displayGroup = _qrviz->getDisplayGroup();
    rviz::Display *display;
    for (int i = 0; i < displayGroup->numDisplays(); i++) {
      display = _qrviz->getDisplayGroup()->getDisplayAt(i);
      if (display->getClassId() == "rviz/PointCloud2" && display->isEnabled()) {
        display->setEnabled(false);
        display->setEnabled(true);
      }
    }
  }
}

void MainWindow::DeteceBtnSlot() {
  //  Py_Initialize();
  //  PyRun_SimpleString("import sys");
  //  PyRun_SimpleString(
  //      "sys.path.append('/home/mith/catkin_qt/src/qt_rviz_demo/scripts')");

  //  PyObject *pModule = PyImport_ImportModule("mmdet3d_client");
  //  if (pModule == NULL) {
  //    PyErr_Print();
  //    qDebug("cannot find module");
  //  }

  //  PyObject *pFunc = PyObject_GetAttrString(pModule, "func");
  //  if (pFunc == NULL) {
  //    PyErr_Print();
  //    qDebug("cannot find function");
  //  }

  //  PyObject *pRet = PyObject_CallObject(pFunc, NULL);

  //  Py_Finalize();
}

void MainWindow::PublishBtnSlot() {
  //  Py_Initialize();
  //  PyRun_SimpleString("import sys");
  //  PyRun_SimpleString(
  //      "sys.path.append('/home/mith/catkin_qt/src/qt_rviz_demo/scripts')");

  //  PyObject *pModule = PyImport_ImportModule("AllNodesInfo");
  //  if (pModule == NULL) {
  //    PyErr_Print();
  //    qDebug("cannot find module");
  //  }

  //  PyObject *pFunc = PyObject_GetAttrString(pModule, "pcl_publish");
  //  if (pFunc == NULL) {
  //    PyErr_Print();
  //    qDebug("cannot find function");
  //  }

  //  PyObject *pRet = PyObject_CallObject(pFunc, NULL);

  //  Py_Finalize();
}

void MainWindow::run_python_code(PyObject *callback) {
  Py_Initialize();

  // call the Python function
  PyObject_CallObject(callback, NULL);

  Py_Finalize();
}

void MainWindow::DataTopicChangedSlot(QString topicName) {
  OnDataTopicChanged(topicName);
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event) {
  if (event->type() == QEvent::MouseButtonPress) {
    if (watched == ui->data_topic_comboBox) {
      QComboBox *comboBox = qobject_cast<QComboBox *>(watched);

      comboBox->clear();

      QMap<QString, QString> all_topics = GetAllTopicsAndTypes();

      QMap<QString, QString>::const_iterator i = all_topics.constBegin();
      while (i != all_topics.constEnd()) {
        comboBox->addItem(i.key());
        ++i;
      }
    }
  }
  return QMainWindow::eventFilter(watched, event);
}

// data tree view
template <typename T> QString MainWindow::typeToString(T type) {
  std::stringstream hexNumber;
  hexNumber << std::setiosflags(std::ios::uppercase) << std::hex
            << type; //大写十六进制
  std::string result = hexNumber.str();
  return QString::fromStdString(result);
}

void MainWindow::InitPC2Model() {
  pc2Model = new QStandardItemModel();
  pc2Model->setHorizontalHeaderLabels(QStringList() << u8"名称" << u8"数值");
  // Header
  QStandardItem *header = new QStandardItem(u8"Header");
  pc2Model->appendRow(header);
  QList<QStandardItem *> items;
  AddTreeViewRow(header, "seq", "", items);
  AddTreeViewRow(header, "stamp", "", items);
  AddTreeViewRow(header->child(1, 0), "sec", "", items);
  AddTreeViewRow(header->child(1, 0), "nsec", "", items);
  AddTreeViewRow(header, "frame_id", "", items);

  // height width
  AddTreeViewRow(pc2Model, "Height", "", items);
  AddTreeViewRow(pc2Model, "Width", "", items);
  // fields
  AddTreeViewRow(pc2Model, "Fields", "", items);
  // is_bigendian point_step row_step is_dense
  AddTreeViewRow(pc2Model, "is_bigendian", "", items);
  AddTreeViewRow(pc2Model, "point_step", "", items);
  AddTreeViewRow(pc2Model, "row_step", "", items);
  AddTreeViewRow(pc2Model, "is_dense", "", items);
}

void MainWindow::UpdateDataTreeView() {}

void MainWindow::AddTreeViewRow(QStandardItem *parentItem, QString name,
                                QString data, QList<QStandardItem *> items) {
  items.append(new QStandardItem(name));
  items.append(new QStandardItem(data));
  parentItem->appendRow(items);
  items.clear();
}

void MainWindow::AddTreeViewRow(QStandardItemModel *parentModel, QString name,
                                QString data, QList<QStandardItem *> items) {
  items.append(new QStandardItem(name));
  items.append(new QStandardItem(data));
  parentModel->appendRow(items);
  items.clear();
}

QMap<QString, QString> MainWindow::GetAllTopicsAndTypes() {
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QMap<QString, QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin();
       it != topic_info.end(); it++) {
    all_topics.insert(it->name.c_str(), it->datatype.c_str());
  }
  return all_topics;
}

void MainWindow::OnDataTopicChanged(QString topicName) {
  QMap<QString, QString> all_topics = GetAllTopicsAndTypes();
  ui->dataType_LineEdit->setText(all_topics[topicName]);

  if (dataTimer != NULL && dataTimer->isActive()) {
    // dataTimer->stop();
    dataTimerThread->terminate();
    qDebug("yes");
  }

  if (all_topics[topicName] == "sensor_msgs/PointCloud2") {
    ui->dataTreeView->setModel(pc2Model);
    //设置水平表头列平均分
    ui->dataTreeView->header()->setSectionResizeMode(QHeaderView::Stretch);
    ui->dataTreeView->expandAll();

    UpdatePC2Data();
    dataTimer->setInterval(200);
    dataTimer->moveToThread(dataTimerThread);
    connect(dataTimer, SIGNAL(timeout()), this, SLOT(UpdatePC2Data()),
            Qt::DirectConnection);
    try {
      connect(dataTimerThread, SIGNAL(started()), dataTimer, SLOT(start()));
      connect(dataTimerThread, SIGNAL(finished()), dataTimer, SLOT(stop()));
    } catch (std::exception e) {
    }

    dataTimerThread->start();

  } else {
    ui->dataTreeView->setModel(emptyModel);
  }
}

void MainWindow::UpdatePC2Data() {

  boost::shared_ptr<sensor_msgs::PointCloud2 const> pc2;
  std::string topic = ui->data_topic_comboBox->currentText().toStdString();
  pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_);

  pc2Model->item(0, 0)->child(0, 1)->setText(
      QString("%1").arg(pc2->header.seq));
  pc2Model->item(0, 0)->child(1, 0)->child(0, 1)->setText(
      QString("%1").arg(pc2->header.stamp.sec));
  pc2Model->item(0, 0)->child(1, 0)->child(1, 1)->setText(
      QString("%1").arg(pc2->header.stamp.nsec));

  pc2Model->item(0, 0)->child(2, 1)->setText(
      typeToString(pc2->header.frame_id));
  pc2Model->item(1, 1)->setText(QString("%1").arg(pc2->height));
  pc2Model->item(2, 1)->setText(QString("%1").arg(pc2->width));
  pc2Model->item(4, 1)->setText(QString("%1").arg(pc2->is_bigendian));
  pc2Model->item(5, 1)->setText(QString("%1").arg(pc2->point_step));
  pc2Model->item(6, 1)->setText(QString("%1").arg(pc2->row_step));
  pc2Model->item(7, 1)->setText(QString("%1").arg(pc2->is_dense));
}

// camera
/**
 * @brief 更新image话题列表
 */
void MainWindow::updateTopicList() {
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(nh_);

  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin();
       it != declared.end(); it++) {
    // qDebug("ImageView::updateTopicList() declared transport '%s'",
    // it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix)) {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QList<QString> selecteds;
  for (QList<QComboBox *>::const_iterator it = camTopicComboBox.constBegin();
       it != camTopicComboBox.constEnd(); it++) {
    selecteds.append((*it)->currentText());
  }

  // fill combo box
  QList<QString> topics =
      getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  qSort(topics);

  for (QList<QComboBox *>::const_iterator it = camTopicComboBox.begin();
       it != camTopicComboBox.end(); it++) {
    (*it)->clear();
  }

  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end();
       it++) {
    QString label(*it);
    label.replace(" ", "/");
    for (QList<QComboBox *>::const_iterator i = camTopicComboBox.begin();
         i != camTopicComboBox.end(); i++) {
      (*i)->addItem(label, QVariant(*it));
    }
  }

  // restore previous selection
  for (int i = 0; i < selecteds.size(); i++) {
    selectTopic(camTopicComboBox.at(i), selecteds.at(i));
  }
}

/**
 * @brief 获取话题
 * @param message_types 消息类型
 * @param message_sub_types
 * @param transports
 * @return 话题集合
 */
QSet<QString> MainWindow::getTopics(const QSet<QString> &message_types,
                                    const QSet<QString> &message_sub_types,
                                    const QList<QString> &transports) {
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin();
       it != topic_info.end(); it++) {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin();
       it != topic_info.end(); it++) {
    if (message_types.contains(it->datatype.c_str())) {
      QString topic = it->name.c_str();

      // add raw topic
      topics.insert(topic);
      // qDebug("ImageView::getTopics() raw topic '%s'",
      // topic.toStdString().c_str());

      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin();
           jt != transports.end(); jt++) {
        if (all_topics.contains(topic + "/" + *jt)) {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
          // qDebug("ImageView::getTopics() transport specific sub-topic
          // '%s'", sub.toStdString().c_str());
        }
      }
    }
    if (message_sub_types.contains(it->datatype.c_str())) {
      QString topic = it->name.c_str();
      int index = topic.lastIndexOf("/");
      if (index != -1) {
        topic.replace(index, 1, " ");
        topics.insert(topic);
        // qDebug("ImageView::getTopics() transport specific sub-topic '%s'",
        // topic.toStdString().c_str());
      }
    }
  }
  return topics;
}

/**
 * @brief 选择话题
 * @param comboBox camera对应的combobox组件
 * @param topic 话题字符串
 */
void MainWindow::selectTopic(QComboBox *comboBox, const QString &topic) {
  int index = comboBox->findText(topic);
  if (index == -1) {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    comboBox->addItem(label, QVariant(topic));
    index = comboBox->findText(topic);
  }
  comboBox->setCurrentIndex(index);
}

/**
 * @brief camera话题改变时调用
 * @param comboBox camera对应的combobox组件
 * @param frame 该camera对应的RatioLayoutedFrame组件
 * @param index 该camera对应的combobox指数
 * @param camSub 该camera的subscriber数组指数
 */
void MainWindow::OnCamTopicChanged(QComboBox *comboBox,
                                   rqt_image_view::RatioLayoutedFrame *frame,
                                   int index, int camSub) {
  conversion_mat_.release();

  camSubcribers[camSub - 1].shutdown();

  // reset image on topic change
  frame->resize(frame->size());
  frame->setImage(QImage());

  QStringList parts = comboBox->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty()) {
    image_transport::ImageTransport it(nh_);
    image_transport::TransportHints hints(transport.toStdString());
    try {
      camSubcribers[camSub - 1] = it.subscribe(
          topic.toStdString(), 1,
          boost::bind(&MainWindow::callbackImage, this, _1, frame));
    } catch (image_transport::TransportLoadException &e) {
      ROS_ERROR("%s", (std::string("Error: ") + e.what()).c_str());
    }
  }
}

/**
 * @brief 订阅image话题的回调函数
 * @param msg
 * @param frame
 */
void MainWindow::callbackImage(const sensor_msgs::Image::ConstPtr &msg,
                               rqt_image_view::RatioLayoutedFrame *frame) {
  try {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("%s", (std::string("Error: ") + e.what()).c_str());
  }

  // Handle rotation
  switch (rotate_state_) {
  case ROTATE_90: {
    cv::Mat tmp;
    cv::transpose(conversion_mat_, tmp);
    cv::flip(tmp, conversion_mat_, 1);
    break;
  }
  case ROTATE_180: {
    cv::Mat tmp;
    cv::flip(conversion_mat_, tmp, -1);
    conversion_mat_ = tmp;
    break;
  }
  case ROTATE_270: {
    cv::Mat tmp;
    cv::transpose(conversion_mat_, tmp);
    cv::flip(tmp, conversion_mat_, 0);
    break;
  }
  default:
    break;
  }

  // image must be copied since it uses the conversion_mat_ for storage
  // which is
  // asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows,
               conversion_mat_.step[0], QImage::Format_RGB888);
  frame->setImage(image);
}

} // namespace qt_rviz_demo
