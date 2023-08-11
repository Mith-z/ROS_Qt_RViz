#include "../include/qt_rviz_demo/record_panel.h"
#include "ui_record_panel.h"

RecordPanel::RecordPanel(QWidget *parent)
    : QWidget(parent), ui(new Ui::RecordPanel) {
  ui->setupUi(this);

  topicMultiComboBox = ui->topic_comboBox;
  // double类型验证器
  QDoubleValidator *validator = new QDoubleValidator(this);
  validator->setNotation(QDoubleValidator::StandardNotation); // 设置符号标记
  // 将验证器设置给录制时长LineEdit
  ui->duration_lineEdit->setValidator(validator);
  //进度条
  ui->record_progressBar->setMinimum(0);
  ui->record_progressBar->setMaximum(100);

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
  //取消按钮
  connect(ui->cancel_Btn, SIGNAL(clicked(bool)), this,
          SLOT(OnCancelBtnClickedSlot()));
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
  double record_duration;

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
  //判断是否输入录制时间
  bool conversionOK;
  double number = ui->duration_lineEdit->text().toDouble(&conversionOK);

  if (conversionOK) {
    qDebug() << "duration: " << number;
    record_duration = number + 0.2;
  } else {
    // 转换失败，提示用户输入正确数字
    QMessageBox::warning(this, "错误", "请输入录制时长");
    return;
  }

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

  RecordRosBag *newRecord = new RecordRosBag(); //创建录制对象
  recordThread = new QThread();
  newRecord->moveToThread(recordThread); //录制移到新线程，防止阻塞

  connect(recordThread, &QThread::started, newRecord, [=]() {
    // newRecord->Record(topicsAndTypes, bag_dir, record_duration);
    this->stopRecord = false;
    newRecord->Record(topicsAndTypes, bag_dir, this);
  });
  recordThread->start();

  recordTimer = new QTimer(); //进度条计时器
  progressValue = 0;
  connect(recordTimer, &QTimer::timeout, this, [=]() {
    progressValue += 1;
    qDebug() << progressValue;
  });
  recordTimer->start(1000); // 每秒触发一次定时器
}

/**
 * @brief RecordPanel::Slot,点击取消按钮
 */
void RecordPanel::OnCancelBtnClickedSlot() {
  // close();
  recordThread->terminate();
  recordTimer->stop();
  this->stopRecord = true;
}

void RecordPanel::showEvent(QShowEvent *event) {
  this->topicMultiComboBox->clear();
  this->topicMultiComboBox->uncheckAll();
  this->topicMultiComboBox->GetSelectModel()->clear();

  ui->dir_lineEdit->clear();
  ui->duration_lineEdit->clear();

  this->progressValue = 0;
  ui->record_progressBar->setValue(progressValue);

  QWidget::showEvent(event);
}

void RecordPanel::closeEvent(QCloseEvent *event) {
  this->mainwindow->setEnabled(true);
  QWidget::closeEvent(event);
}
