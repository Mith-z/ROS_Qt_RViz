#ifndef RATIO_LAYOUTED_FRAME_H
#define RATIO_LAYOUTED_FRAME_H
#include <QFrame>
#include <QImage>
#include <QLayout>
#include <QLayoutItem>
#include <QMutex>
#include <QPainter>
#include <QRect>
#include <QSize>

namespace rqt_image_view {

/**
 * RatioLayoutedFrame is a layout containing a single frame with a fixed aspect
 * ratio. The default aspect ratio is 4:3.
 */
class RatioLayoutedFrame : public QFrame {

  Q_OBJECT

public:
  RatioLayoutedFrame(QWidget *parent, Qt::WindowFlags flags = 0);

  virtual ~RatioLayoutedFrame();

  const QImage &getImage() const;

  QImage getImageCopy() const;

  void setImage(const QImage &image);

  QRect getAspectRatioCorrectPaintArea();

  void resizeToFitAspectRatio();

  void setOuterLayout(QHBoxLayout *outer_layout);

  void setInnerFrameMinimumSize(const QSize &size);

  void setInnerFrameMaximumSize(const QSize &size);

  void setInnerFrameFixedSize(const QSize &size);

signals:

  void delayed_update();

  void mouseLeft(int x, int y);

protected slots:

  void onSmoothImageChanged(bool checked);

protected:
  void setAspectRatio(unsigned short width, unsigned short height);

  void paintEvent(QPaintEvent *event);

private:
  static int greatestCommonDivisor(int a, int b);

  void mousePressEvent(QMouseEvent *mouseEvent);

  QHBoxLayout *outer_layout_;

  QSize aspect_ratio_;

  QImage qimage_;
  mutable QMutex qimage_mutex_;

  bool smoothImage_;
};

} // namespace rqt_image_view

#endif // RATIO_LAYOUTED_FRAME_H
