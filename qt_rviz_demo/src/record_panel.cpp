#include "../include/qt_rviz_demo/record_panel.h"
#include "ui_record_panel.h"

RecordPanel::RecordPanel(QWidget *parent)
    : QWidget(parent), ui(new Ui::RecordPanel) {
  ui->setupUi(this);

  topicMultiComboBox = ui->topic_comboBox;
  //  // double类型验证器
  //  QDoubleValidator *validator = new QDoubleValidator(this);
  //  validator->setNotation(QDoubleValidator::StandardNotation); //
  //  设置符号标记

  //安装事件过滤器
  this->installEventFilter(this);

  // 浏览文件资源管理器
  connect(ui->browse_Btn, &QPushButton::clicked, [=]() {
    QString selectedPath =
        QFileDialog::getExistingDirectory(this, "选择路径", QDir::homePath());

    if (!selectedPath.isEmpty()) {
      ui->dir_lineEdit->setText(selectedPath + "/record.bag");
    }
  });
  //录制按钮
  connect(ui->record_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnRecordBtnClickedSlot()));
  //停止录制按钮
  connect(ui->stop_record_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnStopRecordBtnClickedSlot()));

  recordThread = new QThread();
  recordTimer = new QTimer();
}

RecordPanel::~RecordPanel() {
  delete ui;
  if (recordThread->isRunning()) {
    recordThread->terminate();
  }
  delete recordThread;
}

/**
 * @brief RecordPanel::Slot：点击录制按钮
 */
void RecordPanel::OnRecordBtnClickedSlot() {

  std::string bag_dir;

  //判断是否选择了topic
  if (topicMultiComboBox->currentIndex().count() == 0) {
    QMessageBox::warning(this, "错误", "请选择需要录制的topic");
    return;
  }
  //判断是否选择了路径
  if (ui->dir_lineEdit->text().endsWith(".bag")) {
    bag_dir = ui->dir_lineEdit->text().toStdString();
  } else {
    QMessageBox::warning(this, "错误",
                         "请正确选择bag包的存放路径及文件名，以.bag结尾");
    return;
  }
  //  //判断是否输入录制时间
  //  bool conversionOK;
  //  double number = ui->duration_lineEdit->text().toDouble(&conversionOK);

  //  if (conversionOK) {
  //    qDebug() << "duration: " << number;
  //    record_duration = number + 0.2;
  //  } else {
  //    // 转换失败，提示用户输入正确数字
  //    QMessageBox::warning(this, "错误", "请输入录制时长");
  //    return;
  //  }

  //获取所有的topic和类型
  for (int i : topicMultiComboBox->currentIndex()) {
    topicsAndTypes.insert(topicMultiComboBox->itemText(i),
                          topicMultiComboBox->itemData(i));
  }

  // 判断是否有该路径
  try {
    rosbag::Bag testbag;
    testbag.open(bag_dir, rosbag::bagmode::Write);
    testbag.close();
  } catch (rosbag::BagIOException &e) {
    qDebug() << "Error opening file: " << e.what();
    QMessageBox::warning(this, "错误", "没有该路径！");
    return;
  }

  //开始录制

  ui->record_Btn->setEnabled(false);
  RecordRosBag *newRecord = new RecordRosBag(); //创建录制对象
  newRecord->moveToThread(recordThread); //录制移到新线程，防止阻塞

  connect(recordThread, &QThread::started, newRecord, [=]() {
    // newRecord->Record(topicsAndTypes, bag_dir, record_duration);
    this->stopRecord = false;
    newRecord->Record(topicsAndTypes, bag_dir, this);
  });
  recordThread->start();
  qDebug() << "start record";
  //录制时长计时器
  recordTime = 0.0;
  connect(recordTimer, &QTimer::timeout, this, [=]() {
    recordTime += 0.1;
    ui->record_time_label->setText(QString::number(recordTime));
  });
  recordTimer->start(100); // 每0.1秒触发一次定时器
}

/**
 * @brief slot 点击停止录制按钮时调用
 */
void RecordPanel::OnStopRecordBtnClickedSlot() {
  if (!recordThread->isRunning())
    return;

  ui->record_Btn->setEnabled(true);
  qDebug() << "stop record";
  recordThread->terminate();
  if (recordTimer->isActive())
    recordTimer->stop();
  this->stopRecord = true;
}

void RecordPanel::showEvent(QShowEvent *event) {
  this->topicMultiComboBox->clear();
  this->topicMultiComboBox->uncheckAll();
  this->topicMultiComboBox->GetSelectModel()->clear();

  ui->dir_lineEdit->clear();
  ui->record_time_label->setText("0.0");

  QWidget::showEvent(event);
}

void RecordPanel::closeEvent(QCloseEvent *event) {
  this->mainwindow->setEnabled(true);

  qDebug() << this->recordThread->isRunning();
  OnStopRecordBtnClickedSlot();
  QWidget::closeEvent(event);
}
