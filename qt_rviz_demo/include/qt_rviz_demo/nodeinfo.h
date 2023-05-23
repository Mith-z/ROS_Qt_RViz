#ifndef NODEINFO_H
#define NODEINFO_H

#include <QDebug>
#include <QStandardItemModel>
#include <QString>
#include <QStringList>
#include <QTableView>
#include <QTimer>

#include <Python.h>
#include <fstream>
#include <unistd.h>

namespace Ui {
class MainWindow;
}

class nodeinfo : public QObject {
  Q_OBJECT

public:
  explicit nodeinfo(Ui::MainWindow *mainwindow, int argc, char *argv[]);
  ~nodeinfo();

private slots:
  void updateNodeInfo();
  void GetAllNodesInfo(PyObject *module, int argc, char *argv[]);

private:
  Ui::MainWindow *ui;
  QTimer *timer;

  QTableView *tableView;
  QStandardItemModel *model;
  QStringList AllNodesInfo;

  void InitInfoTable();
};

#endif // NODEINFO_H
