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
  timer->start(1000);
}

nodeinfo::~nodeinfo() {
  Py_Finalize();
  delete timer;
}

void nodeinfo::updateNodeInfo() {}

void nodeinfo::GetAllNodesInfo(PyObject *module, int argc, char *argv[]) {

  PyObject *pFunc = PyObject_GetAttrString(module, "func");
  if (pFunc == NULL) {
    PyErr_Print();
    qDebug("cannot find function");
  }

  PyObject *pRet = PyObject_CallObject(pFunc, NULL);

  char *res;
  PyArg_Parse(pRet, "s", &res);

  QStringList result = QString(QLatin1String(res)).split("\n");
  result.removeLast();

  qDebug() << result << endl;
  //  return result;
}
