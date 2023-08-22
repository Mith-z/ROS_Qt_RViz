#include "../include/qt_rviz_demo/main_window.hpp"
#include "../include/qt_rviz_demo/qrviz.h"
#include "ui_main_window.h"

namespace qt_rviz_demo {

MainWindow::MainWindow(QWidget *parent, int argc, char *argv[])
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  node_ = new nodeinfo(ui, argc, argv);
  initUI();
  initRViz();
  Connects();

  //  ros::Publisher pub = nh_.advertise<qt_rviz_demo::Person>(
  //      "person_topic", 10);
  //  ros::Rate rate(1);       // 设定发布频率

  //  while (ros::ok()) {
  //    qt_rviz_demo::Person person_msg; // 创建一个 Person 类型的消息

  //    // 设置消息中的字段值
  //    person_msg.name = "John";
  //    person_msg.age = 25;
  //    person_msg.gender = "Male";
  //    person_msg.height = 180.0;
  //    person_msg.weight = 75.0;

  //    pub.publish(person_msg); // 发布消息到话题

  //    ros::spinOnce();
  //    rate.sleep();
  //  }
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
  recordBagPanel = new RecordPanel();
  recordBagPanel->setMainWindowWidget(this);
  // menubar

  // toolbar
  toolBarActions = ui->toolBar->actions();
  toolBarActions.at(0)->setChecked(true);
  QActionGroup *group = new QActionGroup(this);
  for (int i = 0; i < toolBarActions.size(); i++) {
    group->addAction(toolBarActions.at(i));
  }

  ui->pause_action->setIcon(QIcon("://images/pause.png"));
  ui->record_action->setIcon(QIcon("://images/record.png"));

  ui->select_action->setIcon(QIcon("://images/classes/Select.png"));
  ui->interact_action->setIcon(QIcon("://images/classes/Interact.png"));
  ui->camera_move_action->setIcon(QIcon("://images/classes/MoveCamera.png"));
  ui->camera_focus_action->setIcon(QIcon("://images/classes/FocusCamera.svg"));

  // camera
  mdiArea = new QMdiArea(ui->camera_widget); // 创建中央部件 QMdiArea
  ui->camera_widget->layout()->addWidget(mdiArea);
  mdiArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  mdiArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  for (int i = 0; i < 4; i++) {
    OnAddCamBtnClickedSlot();
  }

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

  //工具栏tool
  rviz::Tool *toolMoveCamera =
      _qrviz->getToolManager()->addTool("rviz/MoveCamera");
  rviz::Tool *toolSelect = _qrviz->getToolManager()->addTool("rviz/Select");
  rviz::Tool *toolInteract = _qrviz->getToolManager()->addTool("rviz/Interact");
  rviz::Tool *toolCameraFocus =
      _qrviz->getToolManager()->addTool("rviz/FocusCamera");

  connect(ui->interact_action, &QAction::triggered,
          [=]() { _qrviz->getToolManager()->setCurrentTool(toolInteract); });
  connect(ui->camera_move_action, &QAction::triggered,
          [=]() { _qrviz->getToolManager()->setCurrentTool(toolMoveCamera); });
  connect(ui->camera_focus_action, &QAction::triggered,
          [=]() { _qrviz->getToolManager()->setCurrentTool(toolCameraFocus); });
  connect(ui->select_action, &QAction::triggered,
          [=]() { _qrviz->getToolManager()->setCurrentTool(toolSelect); });
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
  this->resizeDocks({typeDock, infoDock, cameraDock}, {300, 200, 225},
                    Qt::Vertical);

  //合并
  this->tabifyDockWidget(typeDock, infoDock);
  typeDock->raise();

  //面板显示控制
  connect(ui->tools_action, &QAction::toggled,
          [=](bool isChecked) { ui->toolBar->setHidden(!isChecked); });
  connect(ui->toolBar, &QToolBar::visibilityChanged, [=]() {
    ui->tools_action->setChecked(!(ui->toolBar->isHidden()));
  }); //工具栏
  connect(ui->type_action, &QAction::toggled,
          [=](bool isChecked) { ui->typeDock->setHidden(!isChecked); });
  connect(ui->typeDock, &QDockWidget::visibilityChanged, [=]() {
    ui->type_action->setChecked(!(ui->typeDock->isHidden()));
  }); //显示
  connect(ui->info_action, &QAction::toggled,
          [=](bool isChecked) { ui->infoDock->setHidden(!isChecked); });
  connect(ui->infoDock, &QDockWidget::visibilityChanged, [=]() {
    ui->info_action->setChecked(!(ui->infoDock->isHidden()));
  }); //信息
  connect(ui->data_action, &QAction::toggled,
          [=](bool isChecked) { ui->dataDock->setHidden(!isChecked); });
  connect(ui->dataDock, &QDockWidget::visibilityChanged, [=]() {
    ui->data_action->setChecked(!(ui->dataDock->isHidden()));
  }); //数据
  connect(ui->camera_action, &QAction::toggled,
          [=](bool isChecked) { ui->cameraDock->setHidden(!isChecked); });
  connect(ui->cameraDock, &QDockWidget::visibilityChanged, [=]() {
    ui->camera_action->setChecked(!(ui->cameraDock->isHidden())); //相机
  });
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
  connect(ui->record_action, SIGNAL(triggered(bool)), this,
          SLOT(RecordActionClickedSlot()));

  // camera
  connect(ui->cam_add_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnAddCamBtnClickedSlot()));
  connect(ui->cam_resize_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnResizeCamBtnClickedSlot()));

  // data treeview
  connect(ui->data_topic_comboBox, SIGNAL(currentIndexChanged(QString)), this,
          SLOT(DataTopicChangedSlot(QString)));
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

//工具栏slot
/**
 * @brief slot 暂停接受点云，画面停在当前帧
 */
void MainWindow::PauseActionClickedSlot() {
  QString fixedFrame = _qrviz->getVisualizationManager()->getFixedFrame();
  if (ui->pause_action->text() == "暂停") {
    ui->pause_action->setText("播放");
    ui->pause_action->setIcon(QIcon("://images/play.png"));
    _qrviz->getVisualizationManager()->getFrameManager()->setFixedFrame("");

  } else {
    ui->pause_action->setText("暂停");
    ui->pause_action->setIcon(QIcon("://images/pause.png"));
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

/**
 * @brief MainWindow::RecordActionClickedSlot
 */
void MainWindow::RecordActionClickedSlot() {
  recordBagPanel->show();
  this->setEnabled(false);

  QMap<QString, QString> alltopics = GetAllTopicsAndTypes();
  for (auto it = alltopics.begin(); it != alltopics.end(); ++it) {
    recordBagPanel->GetTopicComboBox()->addItem(it.key(), it.value());
  }
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
/**
 * @brief MainWindow::InitPC2Model
 */
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
    //清除timer和thread的链接
    disconnect(dataTimer, nullptr, nullptr, nullptr);
    disconnect(dataTimerThread, nullptr, nullptr, nullptr);
  }

  if (all_topics[topicName] == "sensor_msgs/PointCloud2") {
    ui->dataTreeView->setModel(pc2Model);
  }
  //在此处添加else if语句，添加自定义数据消息
  else {
    ui->dataTreeView->setModel(emptyModel);
  }
  //设置水平表头列平均分
  ui->dataTreeView->header()->setSectionResizeMode(QHeaderView::Stretch);
  ui->dataTreeView->expandAll();

  UpdateData(topicName);
  dataTimer->setInterval(200);
  dataTimer->moveToThread(dataTimerThread);
  connect(dataTimer, &QTimer::timeout, [=]() { UpdateData(topicName); });

  try {
    connect(dataTimerThread, SIGNAL(started()), dataTimer, SLOT(start()));
    connect(dataTimerThread, SIGNAL(finished()), dataTimer, SLOT(stop()));
  } catch (std::exception e) {
  }

  dataTimerThread->start();
}

void MainWindow::UpdateData(QString topicName) {
  QMap<QString, QString> all_topics = GetAllTopicsAndTypes();

  if (all_topics[topicName] == "sensor_msgs/PointCloud2") {
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
  //在此处添加else if语句，更新自定义消息数据内容
}

// camera
/**
 * @brief MainWindow::OnAddCamBtnClickedSlot
 */
int cameraIndex = 0; //相机标题index
void MainWindow::OnAddCamBtnClickedSlot() {
  CamMdiSubWindow *newSubWindow = new CamMdiSubWindow;
  mdiArea->addSubWindow(newSubWindow);
  newSubWindow->show();
  //设置标题
  cameraIndex++;
  newSubWindow->setWindowTitle("相机" + QString("%1").arg(cameraIndex));

  connect(newSubWindow->topicComboBox,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          [=](int index) { CamTopicChanged(newSubWindow, index); });

  connect(newSubWindow, &CamMdiSubWindow::comboBoxClicked, [=]() {
    QComboBox *comboBox = newSubWindow->topicComboBox;
    comboBox->clear();
    comboBox->addItem("");

    QMap<QString, QString> all_topics = GetAllTopicsAndTypes();

    QMap<QString, QString>::const_iterator i = all_topics.constBegin();
    while (i != all_topics.constEnd()) {
      if (i.value() == "sensor_msgs/Image" ||
          i.value() == "sensor_msgs/CompressedImage" ||
          i.value() == "sensor_msgs/CameraInfo") {
        comboBox->addItem(i.key());
      }
      ++i;
    }
  });

  qDebug() << "add" << mdiArea->subWindowList().size();
}

/**
 * @brief MainWindow::OnResizeCamBtnClickedSlot
 */
void MainWindow::OnResizeCamBtnClickedSlot() {
  if (mdiArea->subWindowList().isEmpty())
    return;

  int subwindowWidth = mdiArea->size().width() / 4;
  int subwindowHeight = mdiArea->size().height();
  for (QMdiSubWindow *subwindow : mdiArea->subWindowList()) {
    subwindow->resize(subwindowWidth, subwindowHeight);
  }
  for (int i = 0; i < mdiArea->subWindowList().size(); i++) {
    mdiArea->subWindowList()[i]->setGeometry(i % 4 * subwindowWidth,
                                             i / 4 * subwindowHeight,
                                             subwindowWidth, subwindowHeight);

    mdiArea->subWindowList()[i]->setWindowTitle("相机" +
                                                QString("%1").arg(i + 1));
  }
  cameraIndex = mdiArea->subWindowList().size();
  // mdiArea->tileSubWindows();
}

/**
 * @brief 更新image话题列表
 */

void MainWindow::OnCamComboBoxClickedSlot(CamMdiSubWindow *subwindow) {

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

  QString selected;
  selected = subwindow->topicComboBox->currentText();

  // fill combo box
  QList<QString> topics =
      GetTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  qSort(topics);

  subwindow->topicComboBox->clear();

  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end();
       it++) {
    QString label(*it);
    label.replace(" ", "/");

    subwindow->topicComboBox->addItem(label, QVariant(*it));
  }

  SelectTopic(subwindow->topicComboBox, selected);
}

/**
 * @brief 获取话题
 * @param message_types 消息类型
 * @param message_sub_types
 * @param transports
 * @return 话题集合
 */
QSet<QString> MainWindow::GetTopics(const QSet<QString> &message_types,
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
void MainWindow::SelectTopic(QComboBox *comboBox, const QString &topic) {
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
 * @brief MainWindow::CamTopicChanged
 * @param camWindow 当前的相机subwindow
 * @param index subwindow里combobox的index
 */
void MainWindow::CamTopicChanged(CamMdiSubWindow *camWindow, int index) {
  conversion_mat_.release();
  camWindow->camSubscriber.shutdown();

  // reset image on topic change
  camWindow->camFrame->resize(camWindow->camFrame->size());
  camWindow->camFrame->setImage(QImage());

  QStringList parts = camWindow->topicComboBox->itemText(index).split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";
  qDebug() << parts << topic;

  if (!topic.isEmpty()) {
    image_transport::ImageTransport it(nh_);
    image_transport::TransportHints hints(transport.toStdString());
    try {
      camWindow->camSubscriber =
          it.subscribe(topic.toStdString(), 1,
                       boost::bind(&MainWindow::callbackImage, this, _1,
                                   camWindow->camFrame));

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
