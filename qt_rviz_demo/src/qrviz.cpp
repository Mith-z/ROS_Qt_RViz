#include "../include/qt_rviz_demo/qrviz.h"

namespace qt_rviz_demo {

/**
 * @brief 构造函数
 * @param QVBoxLayout，ui中的垂直布局
 */
qrviz::qrviz(QVBoxLayout *qvboxLayout) {
  nullmap.clear();

  this->qVBoxLayout = qvboxLayout;

  renderPanel = new rviz::RenderPanel;
  qvboxLayout->addWidget(renderPanel);
  visualManager = new rviz::VisualizationManager(renderPanel);
  ROS_ASSERT(visualManager != NULL);

  displayGroup = visualManager->getRootDisplayGroup();

  toolManager = visualManager->getToolManager();
  ROS_ASSERT(toolManager != nullptr);

  renderPanel->initialize(visualManager->getSceneManager(), visualManager);
  visualManager->initialize();
  visualManager->removeAllDisplays();
  visualManager->startUpdate();

  visualManager->setFixedFrame("base_link");
}

/**
 * @brief 析构
 */
qrviz::~qrviz() {
  if (qVBoxLayout != nullptr && renderPanel != nullptr) {
    qVBoxLayout->removeWidget(renderPanel);
  }

  if (renderPanel != nullptr)
    delete renderPanel;
  if (visualManager != nullptr)
    delete visualManager;

  /*if (current_tool != nullptr) current_tool = nullptr;
  if (tool_manager_ != nullptr) tool_manager_ = nullptr;*/
}
///
/// \brief 获取RViz显示TreeModel，发出信号
///
void qrviz::GetRVizDisplayTreeModel() {
  rviz::PropertyTreeModel *rvizmodel = visualManager->getDisplayTreeModel();
  QAbstractItemModel *model = rvizmodel;
  emit DisplayTreeModelSignal(model);
}

/**
 * @brief 初始化RViz显示TreeModel
 * @param RViz Display的属性ClassID
 * @param 属性名称name
 * @param enabled 是否显示
 * @param Value QMap映射，这个Display的一些属性和值
 */
void qrviz::InitDisplayTreeModel(QString ClassID, QString name, bool enabled,
                                 QMap<QString, QVariant> Value) {
  //  int num = GetDisplayTreeModelNum(ClassID, name);
  //  if (num == -1) {
  //    rviz::Display *rvizDisplay =
  //        visualManager->createDisplay(ClassID, name, true);
  //    allDisplays.insert(rvizDisplay, name);
  //    emit AddNewDisplaySignal(name);

  //    ROS_ASSERT(rvizDisplay != nullptr);
  //    num = GetDisplayTreeModelNum(ClassID, name);
  //  }

  //  if (!Value.empty()) {
  //    QMap<QString, QVariant>::iterator it;
  //    for (it = Value.begin(); it != Value.end(); it++) {
  //      displayGroup->getDisplayAt(num)->subProp(it.key())->setValue(it.value());
  //    }
  //  }
  //  displayGroup->getDisplayAt(num)->setEnabled(enabled);
  //  visualManager->startUpdate();

  rviz::Display *rvizDisplay =
      visualManager->createDisplay(ClassID, name, true);
  emit AddNewDisplaySignal(name);

  ROS_ASSERT(rvizDisplay != nullptr);

  if (!Value.empty()) {
    QMap<QString, QVariant>::iterator it;
    for (it = Value.begin(); it != Value.end(); it++) {
      rvizDisplay->subProp(it.key())->setValue(it.value());
    }
  }
  rvizDisplay->setEnabled(enabled);
  visualManager->startUpdate();
}

///
/// \brief 根据Display的ClassID(和name)获得Display的序号
/// \param ClassID
/// \return int类型序号
///
int qrviz::GetDisplayTreeModelNum(QString name) {
  int num = -1;
  for (int i = 0; i < displayGroup->numDisplays(); i++) {
    if (displayGroup->getDisplayAt(i) != nullptr) {
      if (name == displayGroup->getDisplayAt(i)->getName()) {
        num = i;
        break;
      }
    }
  }
  return num;
}

int qrviz::GetDisplayTreeModelNum(QString ClassID, QString name) {
  int num = -1;
  for (int i = 0; i < displayGroup->numDisplays(); i++) {
    if (displayGroup->getDisplayAt(i) != nullptr) {
      if (ClassID == displayGroup->getDisplayAt(i)->getClassId() &&
          name == displayGroup->getDisplayAt(i)->getName()) {
        num = i;
        break;
      }
    }
  }
  return num;
}

void qrviz::RemoveDisplay(QString displayName) {
  if (GetDisplayTreeModelNum(displayName) == -1)
    return;

  delete displayGroup->getDisplayAt(GetDisplayTreeModelNum(displayName));
}

} // namespace qt_rviz_demo
