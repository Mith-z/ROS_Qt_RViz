#include <QTimer>
#include <QtGlobal>

#include <rviz/image/ros_image_texture.h>
#include <rviz/ogre_helpers/initialization.h>
#include <rviz/ogre_helpers/qt_ogre_render_window.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureUnitState.h>
#include <OgreViewport.h>

#include "../include/qt_rviz_demo/image_view.h"

using namespace rviz;

ImageView::ImageView(QWidget *parent, const std::string &topic_)
    : QtOgreRenderWindow(parent), texture_it_(nh_), topic(topic_) {
  setAutoRender(false);

  scene_manager_ = ogre_root_->createSceneManager(Ogre::ST_GENERIC);
  //  message_filters::Subscriber<sensor_msgs::Image>
  //  cam1(nh_,"/camera1/color/image_raw",10);
  //  message_filters::Subscriber<sensor_msgs::Image>
  //  cam2(nh_,"/camera2/color/image_raw",10);
  //  message_filters::Subscriber<sensor_msgs::Image>
  //  cam3(nh_,"/camera3/color/image_raw",10);
  //  message_filters::Subscriber<sensor_msgs::Image>
  //  cam4(nh_,"/camera4/color/image_raw",10); typedef
  //  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  //  sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), cam1,
  // cam2,cam3,cam4);
  // sync.registerCallback(boost::bind(&ImageView::callback,this,_1, _2,_3,_4));
  //  std::cout<<"constructor"<<std::endl;
  //  ros::spin();
}
// void ImageView::callback(const sensor_msgs::ImageConstPtr& cam1_,
// 		  const sensor_msgs::ImageConstPtr& cam2_,
// 		  const sensor_msgs::ImageConstPtr& cam3_,
// 		  const sensor_msgs::ImageConstPtr& cam4_){
//          ROS_INFO_STREAM("get asy image!");

// }
ImageView::~ImageView() {
  delete texture_;
  qDebug("delete");
}

void ImageView::showEvent(QShowEvent *event) {
  QtOgreRenderWindow::showEvent(event);

  V_string paths;
  paths.push_back(ros::package::getPath(ROS_PACKAGE_NAME) +
                  "/ogre_media/textures");
  initializeResources(paths);

  if (!scene_manager_->hasCamera("camera_name"))
    setCamera(scene_manager_->createCamera("camera_name"));

  std::string resolved_image = nh_.resolveName("camera_name");
  if (resolved_image == "/image") {
    ROS_WARN("image topic has not been remapped");
  }

  std::stringstream title;
  title << "rviz Image Viewer [" << resolved_image << "]";
  setWindowTitle(QString::fromStdString(title.str()));

  texture_ = new ROSImageTexture();

  try {
    qDebug("origin update");
    texture_->clear();

    texture_sub_.reset(new image_transport::SubscriberFilter());
    texture_sub_->subscribe(texture_it_, topic, 1,
                            image_transport::TransportHints("raw"));
    texture_sub_->registerCallback(
        boost::bind(&ImageView::textureCallback, this, _1));
  } catch (ros::Exception &e) {
    ROS_ERROR("%s", (std::string("Error subscribing: ") + e.what()).c_str());
  }

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
      "camera_Material",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setCullingMode(Ogre::CULL_NONE);
  material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
  material->getTechnique(0)->setLightingEnabled(false);
  Ogre::TextureUnitState *tu =
      material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture_->getTexture()->getName());
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  Ogre::Rectangle2D *rect = new Ogre::Rectangle2D(true);
  rect->setCorners(-1.f, 1.f, 1.f, -1.f);
  rect->setMaterial(material->getName());
  rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  Ogre::AxisAlignedBox aabb;
  aabb.setInfinite();
  rect->setBoundingBox(aabb);

  Ogre::SceneNode *node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  node->attachObject(rect);
  node->setVisible(true);

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(onTimer()));
  timer->start(33);
}

void ImageView::updateTopic() {
  try {
    qDebug("update");
    texture_->clear();

    texture_sub_.reset(new image_transport::SubscriberFilter());
    texture_sub_->subscribe(texture_it_, topic, 1,
                            image_transport::TransportHints("raw"));
    texture_sub_->registerCallback(
        boost::bind(&ImageView::textureCallback, this, _1));
  } catch (ros::Exception &e) {
    ROS_ERROR("%s", (std::string("Error subscribing: ") + e.what()).c_str());
  }
}

void ImageView::onTimer() {
  ros::spinOnce();

  static bool first = true;
  try {
    if (texture_->update()) {
      if (first) {
        first = false;

        resize(texture_->getWidth(), texture_->getHeight());
      }
    }

    ogre_root_->renderOneFrame();
  } catch (UnsupportedImageEncoding &e) {
    ROS_ERROR("%s", e.what());
  }

  if (!nh_.ok()) {
    close();
  }
}

void ImageView::textureCallback(const sensor_msgs::Image::ConstPtr &msg) {
  if (texture_) {
    texture_->addMessage(msg);
  }
}
