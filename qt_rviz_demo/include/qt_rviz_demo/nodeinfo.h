#ifndef NODEINFO_H
#define NODEINFO_H

#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QProcess>
#include <QStandardItemModel>
#include <QString>
#include <QStringList>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

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

  struct NodeInfo {
    QString name;
    int pid;
    double cpuUsage;
    double memUsage;

    bool operator<(const NodeInfo &other) const { return name < other.name; }
  };

  QList<NodeInfo> nodeInfos;
};

#endif // NODEINFO_H
