#ifndef IMAGE_VIEW_H
#define IMAGE_VIEW_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <QVBoxLayout>

#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreTextureUnitState.h>
#include <OgreViewport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/ogre_helpers/initialization.h>
#include <rviz/ogre_helpers/qt_ogre_render_window.h>
#endif

#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
#endif

using namespace rviz;

class ImageView : public QtOgreRenderWindow {
  Q_OBJECT
public:
  std::string topic;
  ImageView(QWidget *parent = nullptr, const std::string &topic_ = NULL);
  ~ImageView() override;
  void updateTopic();

protected:
  void showEvent(QShowEvent *event) override;

private Q_SLOTS:
  void onTimer();

private:
  void textureCallback(const sensor_msgs::Image::ConstPtr &msg);
  // void callback(const sensor_msgs::ImageConstPtr& cam1_,
  //   const sensor_msgs::ImageConstPtr& cam2_,
  //   const sensor_msgs::ImageConstPtr& cam3_,
  //   const sensor_msgs::ImageConstPtr& cam4_);

  Ogre::SceneManager *scene_manager_;
  Ogre::MaterialManager *material_man_;
  Ogre::Camera *camera_;
  ROSImageTexture *texture_;

  ros::NodeHandle nh_;

  image_transport::ImageTransport texture_it_;
  boost::shared_ptr<image_transport::SubscriberFilter> texture_sub_;
};
#endif
