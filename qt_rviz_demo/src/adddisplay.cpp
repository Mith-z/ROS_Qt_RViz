#include "../include/qt_rviz_demo/adddisplay.h"
#include "ui_adddisplay.h"

AddDisplay::AddDisplay(QWidget *parent)
    : QWidget(parent), ui(new Ui::AddDisplay) {
  ui->setupUi(this);
  initUi();

  connect(
      ui->displayType_treeWidget,
      SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)), this,
      SLOT(DisplayWidgetItemChangedSlot(QTreeWidgetItem *, QTreeWidgetItem *)));

  connect(ui->cancel_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnCancelBtnClickedSlot()));
  connect(ui->confirm_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnConfirmBtnClickedSlot()));
}

AddDisplay::~AddDisplay() { delete ui; }

/**
 * @brief 初始化添加显示面板窗口
 */
void AddDisplay::initUi() {
  allDisplayNames.append("Grid");

  QTreeWidgetItem *parent = new QTreeWidgetItem(QStringList() << "rviz");
  parent->setIcon(0, QIcon("://images/classes/Group.png"));
  ui->displayType_treeWidget->addTopLevelItem(parent);
  parent->setExpanded(true);

  Grid = new QTreeWidgetItem(QStringList() << "Grid");
  Grid->setIcon(0, QIcon("://images/classes/Grid.png"));
  parent->addChild(Grid);

  Image = new QTreeWidgetItem(QStringList() << "Image");
  Image->setIcon(0, QIcon("://images/classes/Image.png"));
  parent->addChild(Image);

  PointCloud = new QTreeWidgetItem(QStringList() << "PointCloud");
  PointCloud->setIcon(0, QIcon("://images/classes/PointCloud.png"));
  parent->addChild(PointCloud);

  PointCloud2 = new QTreeWidgetItem(QStringList() << "PointCloud2");
  PointCloud2->setIcon(0, QIcon("://images/classes/PointCloud2.png"));
  parent->addChild(PointCloud2);
}

// Slots
/**
 * @brief slot 选择display时殿用
 * @param current
 * @param previous
 */
void AddDisplay::DisplayWidgetItemChangedSlot(QTreeWidgetItem *current,
                                              QTreeWidgetItem *previous) {
  if (current->text(0) == "rviz")
    ui->confirm_Btn->setEnabled(false);
  else
    ui->confirm_Btn->setEnabled(true);

  ui->name_lineEdit->setText(current->text(0));
  currentChoose = current->clone();
}

/**
 * @brief slot 取消按钮
 */
void AddDisplay::OnCancelBtnClickedSlot() { this->close(); }

/**
 * @brief slot 点击确认按钮时调用
 */
void AddDisplay::OnConfirmBtnClickedSlot() {
  qDebug() << HasRepeatedDisplayName(ui->name_lineEdit->text());
  if (HasRepeatedDisplayName(ui->name_lineEdit->text())) {
    ui->name_lineEdit->setText("名称重复！请重新输入名称");
  } else {
    emit(ChooseDisplaySignal(currentChoose, ui->name_lineEdit->text()));
    this->close();
  }
}

void AddDisplay::AddNewDisplaySlot(QString name) {
  allDisplayNames.append(name);

  qDebug() << allDisplayNames.size();
  for (int i = 0; i < allDisplayNames.size(); i++) {
    qDebug() << allDisplayNames.at(i);
  }
}

bool AddDisplay::HasRepeatedDisplayName(QString newName) {
  for (int i = 0; i < allDisplayNames.size(); i++) {
    if (allDisplayNames.at(i) == newName)
      return true;
  }
  return false;
}
