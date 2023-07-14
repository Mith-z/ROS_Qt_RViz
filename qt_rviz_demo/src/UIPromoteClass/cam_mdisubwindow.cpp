#include "./include/qt_rviz_demo/UIPromoteClass/cam_mdisubwindow.h"

CamMdiSubWindow::CamMdiSubWindow(QWidget *parent) : QMdiSubWindow(parent) {
  this->setWindowFlags(this->windowFlags() &
                       ~Qt::WindowMinMaxButtonsHint); // 移除最小化和最大化按钮
  this->setWindowIcon(QIcon("://images/classes/Camera.png"));
  this->setContentsMargins(0, 0, 0, 0);

  camWidget = new QWidget();
  camTopicWidget = new QWidget();

  topicLabel = new QLabel();

  topicComboBox = new QComboBox();
  topicComboBox->setObjectName("combo");
  topicComboBox->installEventFilter(this);

  camFrame = new rqt_image_view::RatioLayoutedFrame();
  camFrame->setFrameShape(QFrame::Shape::StyledPanel);
  camFrame->setFrameShadow(QFrame::Shadow::Raised);

  QHBoxLayout *camTopicWidget_layout = new QHBoxLayout(camTopicWidget);
  camTopicWidget_layout->addWidget(topicLabel, 1);
  camTopicWidget_layout->addWidget(topicComboBox, 6);
  topicLabel->setText("Topic: ");

  QVBoxLayout *camWidget_layout = new QVBoxLayout(camWidget);
  camWidget_layout->addWidget(camTopicWidget, 1);
  camWidget_layout->addWidget(camFrame, 6);

  camTopicWidget_layout->setContentsMargins(0, 0, 0, 0);
  camWidget_layout->setContentsMargins(0, 0, 0, 0);

  this->setWidget(camWidget);
  this->resize(300, 200);
}

CamMdiSubWindow::~CamMdiSubWindow() {}

/**
 * @brief 重写事件处理器
 * @param watched 鼠标获取的对象
 * @param event 鼠标点击事件
 * @return
 */
bool CamMdiSubWindow::eventFilter(QObject *watched, QEvent *event) {
  if (event->type() == QEvent::MouseButtonPress) {
    if (watched == topicComboBox) {
    }
  }
  return QMdiSubWindow::eventFilter(watched, event);
}
