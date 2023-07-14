#ifndef CAMMDISUBWINDOW_H
#define CAMMDISUBWINDOW_H

#include "ratio_layouted_frame.h"
#include <QtWidgets>

#include <image_transport/image_transport.h>

class CamMdiSubWindow : public QMdiSubWindow {
  Q_OBJECT
public:
  CamMdiSubWindow(QWidget *parent = nullptr);
  ~CamMdiSubWindow();

signals:
  void closed();

protected:
  bool eventFilter(QObject *watched, QEvent *event);
  void closeEvent(QCloseEvent *event) override {
    // 在关闭窗口前执行你的逻辑
    qDebug() << "CamMdiSubWindow is closing.";

    // 调用基类的 closeEvent 函数继续处理关闭事件
    QMdiSubWindow::closeEvent(event);
    deleteLater();
  }

public:
  QWidget *camWidget;
  QWidget *camTopicWidget;
  QLabel *topicLabel;
  QComboBox *topicComboBox;
  rqt_image_view::RatioLayoutedFrame *camFrame;

  image_transport::Subscriber camSubscriber;
};
#endif // CAMMDISUBWINDOW_H
