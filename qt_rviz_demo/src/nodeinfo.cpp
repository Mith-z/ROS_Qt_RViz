#include "../include/qt_rviz_demo/nodeinfo.h"
#include "ui_main_window.h"

nodeinfo::nodeinfo(Ui::MainWindow *mainwindow, int argc, char *argv[])
    : ui(mainwindow) {
  // updateNodeInfo();

  //初始化python环境
  Py_Initialize();
  // 设置命令行参数
  PySys_SetArgv(argc, argv);

  PyRun_SimpleString("import sys");
  PyRun_SimpleString(
      "sys.path.append('/home/mith/catkin_qt/src/qt_rviz_demo/scripts')");

  //调用ALLNodesInfo模块
  PyObject *pModule = PyImport_ImportModule("AllNodesInfo");
  if (pModule == NULL) {
    PyErr_Print();
    qDebug("cannot find module");
  }

  timer = new QTimer();
  connect(timer, &QTimer::timeout, this,
          [=]() { GetAllNodesInfo(pModule, argc, argv); });
  timer->start(2000);
  this->InitInfoTable();
}

nodeinfo::~nodeinfo() {
  Py_Finalize();
  delete timer;
}

void nodeinfo::InitInfoTable() {
  // 创建表格视图和表格模型
  model = new QStandardItemModel();
  ui->node_info_tableView->setModel(model);

  // 设置表格模型的列数、表头名称和水平对齐方式
  model->setColumnCount(5);
  model->setHeaderData(0, Qt::Horizontal, "PID");
  model->setHeaderData(1, Qt::Horizontal, "进程名");
  model->setHeaderData(2, Qt::Horizontal, "CPU%");
  model->setHeaderData(3, Qt::Horizontal, "MEM%");
  model->setHeaderData(4, Qt::Horizontal, "线程数");

  //居中对齐
  ui->node_info_tableView->horizontalHeader()->setDefaultAlignment(
      Qt::AlignLeft);
  ui->node_info_tableView->verticalHeader()->hide();
  ui->node_info_tableView->horizontalHeader()->setStretchLastSection(true);

  //设置列宽随内容改变，进程名可拖拽
  ui->node_info_tableView->horizontalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
  ui->node_info_tableView->horizontalHeader()->setSectionResizeMode(
      1, QHeaderView::Interactive);
  ui->node_info_tableView->horizontalHeader()->resizeSection(1, 130);
}

void nodeinfo::updateNodeInfo() {}

void nodeinfo::GetAllNodesInfo(PyObject *module, int argc, char *argv[]) {

  PyObject *pFunc = PyObject_GetAttrString(module, "all_nodes_info");
  if (pFunc == NULL) {
    PyErr_Print();
    qDebug("cannot find function");
  }

  PyObject *pRet = PyObject_CallObject(pFunc, NULL);

  char *result;
  PyArg_Parse(pRet, "s", &result);

  AllNodesInfo = QString(QLatin1String(result)).split("\n");
  AllNodesInfo.removeLast();
  //更新数据
  model->removeRows(0, model->rowCount());
  for (int i = 0; i < AllNodesInfo.size(); i++) {
    QStringList node = AllNodesInfo[i].split(" ");
    node.removeLast();
    qSwap(node[0], node[1]);
    for (int j = 0; j < node.size(); j++) {
      model->setItem(i, j, new QStandardItem(node[j]));
    }
  }
  model->sort(2, Qt::DescendingOrder);
}
