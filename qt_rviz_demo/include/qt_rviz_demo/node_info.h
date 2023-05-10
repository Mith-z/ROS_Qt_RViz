#ifndef NODE_INFO_H
#define NODE_INFO_H

#include <QMainWindow>
#include <QStandardItem>
#include <QTime>
#include <QTimer>

#include <Python.h>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string.h>
#include <string>
#include <vector>

#include <ros/master.h>

#include <sys/sysinfo.h>

#define CPU 0
#define MEM 1
#define CLK_TCK 100
#define BUF_SIZE 1024

// 向后偏移14项，正好是utime
#define PROCESS_ITEM 14

using namespace std;

namespace Ui {
class MainWindow;
}

namespace qt_rviz_demo {

class NodeInfo : public QObject {
  Q_OBJECT
public:
  explicit NodeInfo(Ui::MainWindow *mainwindow);
  ~NodeInfo();

  QString GetProcessPIDbyName(char *process_name);
  QStringList GetAllNodePid();

private slots:
  void UpdateStat();
  void button();

private:
  bool isDigit(char *str);
  std::map<int, std::string> *GetUsernameDict();
  std::string GetProperUnit(int num);

  void InitTable();
  void GetCPUStat();
  void GetMEMStat();
  void GetProcessInfo(int total_diff);
  void ReFocus();

  struct CpuUsed {
    unsigned long sys_used;
    float proc_used;
  };
  struct CpuStat {
    char name[8];       // 设备名，一般值为pu
    unsigned long user; // 用户态的CPU时间（单位:0.01s）
    unsigned long nice; // nice值为负的进程所占用的CPU时间（单位:0.01s）
    unsigned long system; // 处于核心态的运行时间（单位:0.01s）
    unsigned long idle; // 除IO等待时间意外其他等待时间（单位:0.01s）
  };
  struct MemInfo {
    unsigned long total;
    unsigned long free;
    unsigned long avaliable;
  };

  static void GetSystemCpuStat(CpuStat *s);
  static unsigned long GetProcessCpuTime(char *pid);
  CpuUsed GetCpuUsed(char *pid);

  static void GetSystemMeminfo(MemInfo *info);
  static unsigned long GetProcessMeminfo(int pid);
  int GetSystemMemUsed(int &total);
  int GetProcessMemUsed(void);

  Ui::MainWindow *ui;

  int total_cpu_time;
  int total_user_time;
  int total_sys_time;
  int total_idle_time;
  bool MemState;
  bool CpuState;
  int num_processes;
  int last_pid;
  int mem_size;

  QStringList nodePIDs;

  QTimer *timer;
  QStandardItemModel *model;
  QStandardItemModel *cpu_model;
  QStandardItemModel *mem_model;

  std::map<int, int> *process_map;
  std::map<int, std::string> *username_dict;

  struct dirent *ptr = nullptr;
  DIR *dir = nullptr;
};

} // namespace qt_rviz_demo
#endif // NODE_INFO_H
