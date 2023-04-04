#ifndef ADDDISPLAY_H
#define ADDDISPLAY_H

#include <QCheckBox>
#include <QTreeWidgetItem>
#include <QWidget>

#include "qrviz.h"

namespace Ui {
class AddDisplay;
}

class AddDisplay : public QWidget {
  Q_OBJECT

public:
  explicit AddDisplay(QWidget *parent = 0);
  ~AddDisplay();

  void initUi();
  bool HasRepeatedDisplayName(QString newName);

  QTreeWidgetItem *currentChoose;
  QList<QString> allDisplayNames;

signals:
  void ChooseDisplaySignal(QTreeWidgetItem *currentChoose,
                           QString currentChooseName);

public slots:
  void OnCancelBtnClickedSlot();
  void OnConfirmBtnClickedSlot();
  void DisplayWidgetItemChangedSlot(QTreeWidgetItem *current,
                                    QTreeWidgetItem *previous);
  void AddNewDisplaySlot(QString name);

private:
  Ui::AddDisplay *ui;

  QTreeWidgetItem *Grid = NULL;
  QTreeWidgetItem *PointCloud = NULL;
  QTreeWidgetItem *PointCloud2 = NULL;
  QTreeWidgetItem *Image = NULL;
  QTreeWidgetItem *Camera = NULL;
};

#endif // ADDDISPLAY_H
