#ifndef QRVIZ_H
#define QRVIZ_H

#include "adddisplay.h"

#include <QAbstractItemModel>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QtDebug>

#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>

namespace qt_rviz_demo {

class qrviz : public QThread {
  Q_OBJECT
public:
  qrviz(QVBoxLayout *qvboxLayout);
  ~qrviz();

  void GetRVizDisplayTreeModel();
  void InitDisplayTreeModel(QString ClassID, QString name, bool enabled,
                            QMap<QString, QVariant> Value);
  int GetDisplayTreeModelNum(QString ClassID);
  int GetDisplayTreeModelNum(QString ClassID, QString name);

  void Sub_Image();
signals:
  void DisplayTreeModelSignal(QAbstractItemModel *model);
  void AddNewDisplaySignal(QString name);

public slots:

private:
  // rviz显示容器
  rviz::RenderPanel *renderPanel;
  rviz::VisualizationManager *visualManager;
  rviz::DisplayGroup *displayGroup;
  // ImageView *imageView;
  // rviz控制工具
  rviz::Tool *currentTool;
  rviz::ToolManager *toolManager;

  QMap<QString, QVariant> nullmap;
  QVBoxLayout *qVBoxLayout;
};
} // namespace qt_rviz_demo

#endif // QRVIZ_H
