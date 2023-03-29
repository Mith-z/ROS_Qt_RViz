#include "../include/qt_rviz_demo/main_window.hpp"
#include "../include/qt_rviz_demo/qrviz.h"
#include "ui_main_window.h"
#include <QDebug>
#include <QtDebug>

namespace qt_rviz_demo {

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  initUI();
  initRViz();

  addDisplayPanel = new AddDisplay();

  Connects();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::initUI() {
  initializeDockWidgets();

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
}

/**
 * @brief 链接各个Signal和Slot
 */
void MainWindow::Connects() {
  connect(ui->add_display_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnAddDisplayBtnClickedSlot()));
  connect(_qrviz, SIGNAL(AddNewDisplaySignal(QString)), addDisplayPanel,
          SLOT(AddNewDisplaySlot(QString)));
  connect(addDisplayPanel,
          SIGNAL(ChooseDisplaySignal(QTreeWidgetItem *, QString)), this,
          SLOT(AddNewDisplaySlot(QTreeWidgetItem *, QString)));

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
}

/**
 * @brief 添加Display按钮
 */
void MainWindow::OnAddDisplayBtnClickedSlot() { addDisplayPanel->show(); }

/**
 * @brief MainWindow::AddNewDisplaySlot
 * @param name
 */
void MainWindow::AddNewDisplaySlot(QTreeWidgetItem *newDisplay, QString name) {
  QMap<QString, QVariant> Value;
  Value.clear();
  qDebug() << "添加Disaplay  " + name;
  _qrviz->InitDisplayTreeModel("rviz/" + newDisplay->text(0), name, true,
                               Value);
}

void MainWindow::Cam1TopicEditSlot(QString topic) {
  //  if (topic != NULL) {
  //    try {
  //      qDebug("test");

  //      if (ui->cam1_img_widget->findChild<ImageView *>("image") != NULL)
  //        delete ui->cam1_img_widget->findChild<ImageView *>("image");

  //      ImageView *imageView =
  //          new ImageView(ui->cam1_img_widget, topic.toStdString());

  //      imageView->setObjectName("image");
  //      imageView->setParent(ui->cam1_img_widget);

  //      if (ui->cam1_img_widget->layout() != NULL)
  //        delete ui->cam1_img_widget->layout();

  //      QGridLayout *layout = new QGridLayout(ui->cam1_img_widget);
  //      layout->addWidget(imageView);
  //      ui->cam1_img_widget->setLayout(layout);

  //    } catch (Ogre::ItemIdentityException &e) {
  //      ROS_ERROR("%s", (std::string("Error subscribing: ") +
  //      e.what()).c_str());
  //    }
  //  }
}

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
          // qDebug("ImageView::getTopics() transport specific sub-topic '%s'",
          // sub.toStdString().c_str());
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
