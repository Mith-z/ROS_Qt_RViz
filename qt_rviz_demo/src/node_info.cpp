#include "../include/qt_rviz_demo/node_info.h"
#include "ui_main_window.h"
#include <QDebug>
#include <stdio.h>

namespace qt_rviz_demo {

NodeInfo::NodeInfo(Ui::MainWindow *mainwindow) : ui(mainwindow) {
  this->timer = new QTimer(this);
  this->cpu_model = new QStandardItemModel();
  this->mem_model = new QStandardItemModel();
  this->MemState = false;
  this->CpuState = true;
  this->model = this->cpu_model;
  this->InitTable();
  ui->node_info_frame->add_data(0.1);
  process_map = new std::map<int, int>();
  username_dict = GetUsernameDict();
  timer->start(2000);
  this->total_cpu_time = 0;
  this->total_idle_time = 0;
  this->last_pid = -1;
  this->num_processes = 0;
  this->nodePIDs = GetAllNodePid();
  GetCPUStat();
  GetProcessInfo(0);
  connect(timer, SIGNAL(timeout()), this, SLOT(UpdateStat()));
  connect(ui->pushButton, SIGNAL(clicked(bool)), this, SLOT(button()));
  //  int total;
  //  GetSystemMemUsed(total);
  //  cout << total << endl;
  //  cout << GetProcessMemUsed() << endl;
  //  cout << float(GetProcessMemUsed()) / float(total) << endl;
}

NodeInfo::~NodeInfo() {
  if (timer->isActive()) {
    timer->stop();
  }
  delete timer;
  delete process_map;
  delete username_dict;
  delete cpu_model;
  delete mem_model;
  delete model;
  delete ui;
}

// 解析/proc/stat文件，获取CPU信息
void NodeInfo::GetSystemCpuStat(CpuStat *s) {
  memset(s, 0, sizeof(CpuStat));

  char buf[256];
  FILE *fd = fopen("/proc/stat", "r");
  assert(fd != nullptr);
  fgets(buf, sizeof(buf), fd);
  sscanf(buf, "%s  %lu %lu %lu %lu", s->name, &s->user, &s->nice, &s->system,
         &s->idle);
  fclose(fd);
  // fprintf(stdout, "%s: %lu %lu %lu %lu\n", s->name, s->user, s->nice,
  // s->system, s->idle);
}

// 获取/proc/{pid}/stat中CPU的使用信息，主要是
// (14) utime  %lu
// (15) stime  %lu
// (16) cutime  %ld
// (17) cstime  %ld
unsigned long NodeInfo::GetProcessCpuTime(char *pid) {
  // 计算当前进程对应的文件名
  char filename[64];
  snprintf(filename, sizeof(filename) - 1, "/proc/%s/stat", pid);

  char buf[1024];
  FILE *fd = fopen(filename, "r");
  assert(fd != nullptr);
  fgets(buf, sizeof(buf), fd);
  // fprintf(stdout, "%s: %s", filename, buf);
  fclose(fd);

  // 向后偏移，直至utime项
  int count = 1;
  const char *p = buf;
  while (*p != '\0') {
    if (*p++ == ' ') {
      count++;
      if (count == PROCESS_ITEM)
        break;
    }
  }
  if (*p == '\0')
    return 0;

  unsigned long utime = 0;  // user time
  unsigned long stime = 0;  // kernel time
  unsigned long cutime = 0; // all user time
  unsigned long cstime = 0; // all dead time
  sscanf(p, "%ld %ld %ld %ld", &utime, &stime, &cutime, &cstime);
  // fprintf(stdout, "proc[%d]: %lu %lu %lu %lu\n", pid, utime, stime, cutime,
  // cstime);
  return (utime + stime + cutime + cstime);
}

// 获取CPU利用率，如CPU利用率为60%，则返回60
NodeInfo::CpuUsed NodeInfo::GetCpuUsed(char *pid) {
  CpuUsed used;
  used.sys_used = used.proc_used = 0;

  // int pid = getpid();
  CpuStat s1, s2;
  GetSystemCpuStat(&s1);
  unsigned long proc_time1 = GetProcessCpuTime(pid);

  usleep(2000 * 1000);

  GetSystemCpuStat(&s2);
  unsigned long proc_time2 = GetProcessCpuTime(pid);

  // 计算CPU利用率：使用时间/总时间
  unsigned int sys_used1 = s1.user + s1.nice + s1.system;
  unsigned int sys_used2 = s2.user + s2.nice + s2.system;
  unsigned int sys_total1 = sys_used1 + s1.idle;
  unsigned int sys_total2 = sys_used2 + s2.idle;

  if (sys_total1 < sys_total2) {
    used.sys_used = 100 * (sys_used2 - sys_used1) / (sys_total2 - sys_total1);
    // 多线程，需要乘CPU core数量
    used.proc_used = get_nprocs() * 100 * float(proc_time2 - proc_time1) /
                     float(sys_total2 - sys_total1);
  }

  return used;
}

void NodeInfo::GetSystemMeminfo(MemInfo *info) {
  memset(info, 0, sizeof(MemInfo));

  char buf[128];
  char name[64];
  char unit[64];
  FILE *fd = fopen("/proc/meminfo", "r");
  assert(fd != nullptr);
  fgets(buf, sizeof(buf), fd);
  sscanf(buf, "%s %u %s", name, &info->total, unit);
  fgets(buf, sizeof(buf), fd);
  sscanf(buf, "%s %u %s", name, &info->free, unit);
  fgets(buf, sizeof(buf), fd);
  sscanf(buf, "%s %u %s", name, &info->avaliable, unit);
  fclose(fd);
  // fprintf(stdout, "mem: %u %u %u\n", info->total, info->free,
  // info->avaliable);
}

unsigned long NodeInfo::GetProcessMeminfo(int pid) {
  // 计算当前进程对应的文件名
  char filename[64];
  snprintf(filename, sizeof(filename) - 1, "/proc/%d/statm", pid);

  char buf[256];
  FILE *fd = fopen(filename, "r");
  assert(fd != nullptr);
  fgets(buf, sizeof(buf), fd);
  fclose(fd);

  unsigned long size = 0;
  unsigned long resident = 0;
  sscanf(buf, "%lu %lu", &size, &resident);
  // fprintf(stdout, "proc mem: %u %u\n", size, resident);
  return resident * 6;
}

// 获取系统已使用的内存，单位MB。total为输出参数，表示总内存
int NodeInfo::GetSystemMemUsed(int &total) {
  MemInfo info;
  GetSystemMeminfo(&info);
  total = info.total / 1024;
  return (total - info.avaliable / 1024);
}

// 获取本进程已使用的内存，单位MB
int NodeInfo::GetProcessMemUsed(void) {
  unsigned long mem_used = GetProcessMeminfo(getpid());
  return ((mem_used < 1024) ? 1 : mem_used / 1024);
}

void NodeInfo::InitTable() {
  // ui->tableView->setSortingEnabled(true);

  model = this->cpu_model;
  ui->node_info_tableView->setModel(model);
  // ui->tableView->setModel(this->proxy_model);
  // ui->tableView->sortByColumn(1, Qt::DescendingOrder);
  model->setColumnCount(6);
  model->setHeaderData(0, Qt::Horizontal, "  PID");
  model->setHeaderData(1, Qt::Horizontal, "进程名称");
  model->setHeaderData(2, Qt::Horizontal, "  % CPU");
  model->setHeaderData(3, Qt::Horizontal, " CPU时间");
  model->setHeaderData(4, Qt::Horizontal, "  % MEM");
  //    model->setHeaderData(3, Qt::Horizontal, "    线程");
  //    model->setHeaderData(4, Qt::Horizontal, "闲置唤醒");
  model->setHeaderData(5, Qt::Horizontal, "用户");
  ui->node_info_tableView->horizontalHeader()->setDefaultAlignment(
      Qt::AlignLeft);
  model->sort(1, Qt::DescendingOrder);

  ui->node_info_tableView->verticalHeader()->hide();
  ui->node_info_tableView->setColumnWidth(0, 130);
  ui->node_info_tableView->horizontalHeader()->setStretchLastSection(true);
  ui->node_info_tableView->horizontalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
}

QStringList NodeInfo::GetAllNodePid() {
  Py_Initialize();
  PyRun_SimpleString("import sys");
  PyRun_SimpleString(
      "sys.path.append('/home/mith/catkin_qt/src/qt_rviz_demo/scripts')");

  PyObject *pModule = PyImport_ImportModule("calltest");
  if (pModule == NULL) {
    PyErr_Print();
    qDebug("cannot find module");
  }

  PyObject *pFunc = PyObject_GetAttrString(pModule, "func");
  if (pFunc == NULL) {
    PyErr_Print();
    qDebug("cannot find function");
  }

  PyObject *pRet = PyObject_CallObject(pFunc, NULL);

  char *res;
  PyArg_Parse(pRet, "s", &res);
  Py_Finalize();

  QStringList result = QString(QLatin1String(res)).split("\n");
  result.removeLast();

  return result;
}

void NodeInfo::GetCPUStat() {
  std::ifstream input;
  input.open("/proc/stat", std::ios_base::in);
  int total_diff = 0;
  if (input.is_open()) {
    std::string cpu;
    int user, nice, system, idle, iowait, irq, softirq, stealstolen, guest;
    input >> cpu >> user >> nice >> system >> idle >> iowait >> irq >>
        softirq >> stealstolen >> guest;
    input.close();
    int total_cpu_time = user + nice + system + idle + irq + softirq + iowait;
    if (this->total_cpu_time != 0) {
      total_diff = total_cpu_time - this->total_cpu_time;
      int idle_diff = idle - this->total_idle_time;
      int user_diff = user - this->total_user_time;
      int sys_diff = system - this->total_sys_time;
      float total_rate = (total_diff - idle_diff) * 1.0 / total_diff;
      float idle_rate = idle_diff * 1.0 / total_diff;
      float user_rate = user_diff * 1.0 / total_diff;
      float sys_rate = sys_diff * 1.0 / total_diff;
      //      ui->left_1->setText(tr("系统:\t          %1%")
      //                              .arg(QString::number(sys_rate * 100, 'f',
      //                              2)));
      //      ui->left_2->setText(tr("用户:\t          %1%")
      //                              .arg(QString::number(user_rate * 100, 'f',
      //                              2)));
      //      ui->left_3->setText(tr("空闲:\t          %1%")
      //                              .arg(QString::number(idle_rate * 100, 'f',
      //                              2)));
      //      ui->right_1->setText(tr("总利用率:           %1%")
      //                               .arg(QString::number(total_rate * 100,
      //                               'f', 1)));
      //      ui->right_1->setStyleSheet("QLabel {\n	color:rgb(242, 21,
      //      21)\n}");
      ui->node_info_frame->add_data(total_rate);
    }
    this->total_cpu_time = total_cpu_time;
    this->total_idle_time = idle;
    this->total_sys_time = system;
    this->total_user_time = user;
  } else {
    qDebug() << "Cannot open file" << endl;
  }
  GetProcessInfo(total_diff);
}

void NodeInfo::GetMEMStat() {
  std::ifstream input;
  input.open("/proc/meminfo", std::ios_base::in);
  if (input.is_open()) {
    int mem_total, mem_free, mem_used, mem_cached;
    int swap_total, swap_free, swap_used;
    std::string tmp;
    char buffer[100];
    int values[45];
    int i = 0;
    while (i < 45) {
      input >> tmp >> values[i];
      input.getline(buffer, 100);
      i++;
    }
    mem_total = values[0];
    mem_free = values[1];
    mem_used = mem_total - mem_free;
    mem_cached = values[4];
    swap_total = values[14];
    swap_free = values[15];
    swap_used = swap_total - swap_free;
    float rate = mem_used * 100.0 / mem_total;
    this->mem_size = mem_total;

    ui->node_info_frame->add_data(rate / 100);
    input.close();
  }
  GetProcessInfo(0);
}

QString NodeInfo::GetProcessPIDbyName(char *process_name) {
  DIR *dir;
  struct dirent *ptr;
  FILE *fp;
  char filepath[50];
  char cur_process_name[50];
  char buf[BUF_SIZE];

  dir = opendir("/proc");
  if (dir != NULL) {
    while ((ptr = readdir(dir)) != NULL) {
      if ((strcmp(ptr->d_name, ".") == 0) || (strcmp(ptr->d_name, "..") == 0))
        continue;
      if (ptr->d_type != DT_DIR)
        continue;

      sprintf(filepath, "/proc/%s/status", ptr->d_name);
      fp = fopen(filepath, "r");
      if (fp != NULL) {
        if (fgets(buf, BUF_SIZE - 1, fp) == NULL) {
          fclose(fp);
          continue;
        }
        sscanf(buf, "%*s %s", cur_process_name);

        if (!strcmp(process_name, cur_process_name)) {
          return QString(QLatin1String(ptr->d_name));
        }

        fclose(fp);
      }
    }
    closedir(dir);
  }
  return "";
}

void NodeInfo::GetProcessInfo(int total_diff) {
  std::ifstream input;
  dir = opendir("/proc");
  int i = 0;
  int cnt = 0;
  while ((ptr = readdir(dir)) != NULL) {
    if (isDigit(ptr->d_name)) {

      //      char filename[40] = "/proc/";
      //      strcat(filename, ptr->d_name);
      //      input.open(strcat(filename, "/stat"));

      if (nodePIDs.contains(QString(QLatin1String(ptr->d_name)))) {
        QString filename = "/proc/";
        input.open((filename + QString(QLatin1String(ptr->d_name)) + "/stat")
                       .toLatin1()
                       .data());
      }

      if (input.is_open()) {
        cnt++;
        std::string name; // process name
        int pid;          // process id
        char state;       // process state : "RSDZTW"
        int ppid, pgid, sid, tty_nr, tty_pgrp, flags, min_flt;
        int cmin_flt, maj_flt, cmaj_flt, priority, nice;
        long utime, stime, cutime, cstime;
        int num_threads, zero;
        unsigned long long start_time;
        input >> pid >> name >> state >> ppid >> pgid >> sid >> tty_nr >>
            tty_pgrp >> flags >> min_flt >> cmin_flt >> maj_flt >> cmaj_flt >>
            utime >> stime >> cutime >> cstime >> priority >> nice >>
            num_threads >> zero >> start_time;
        // discard the parentheses
        char *tmp = (char *)name.c_str();
        int len = strlen(tmp);
        tmp[len - 1] = '\0';

        model->setItem(i, 0, new QStandardItem(ptr->d_name));
        model->item(i, 0)->setTextAlignment(Qt::AlignCenter);
        model->setItem(i, 1, new QStandardItem(tmp + 1));

        if (this->CpuState == true) {
          // compute processes' CPU rate
          if (total_diff == 0) {
            model->setItem(i, 2, new QStandardItem("00.00%"));
          } else {
            int total_process = utime + stime + cutime + cstime;
            if (process_map->find(pid) != process_map->end()) {
              int total_pro_diff = total_process - (*process_map)[pid];
              float rate;
              rate = (QString(QLatin1String(ptr->d_name)) ==
                      QString::number(getpid()))
                         ? total_pro_diff * 0.5 / total_diff
                         : total_pro_diff * 4.0 / total_diff;

              QString temp = tr("%1%").arg(QString::number(rate * 100, 'f', 2));
              if (temp.length() < 6) {
                temp = tr("0%1").arg(temp);
              }
              model->setItem(i, 2, new QStandardItem(temp));

            } else {
              model->setItem(i, 2, new QStandardItem("00.00%"));
            }
            (*process_map)[pid] = total_process;
          }
          model->item(i, 2)->setTextAlignment(Qt::AlignCenter);
          // process runing time
          std::ifstream uptime;
          uptime.open("/proc/uptime");
          if (uptime.is_open()) {
            float total_time;
            uptime >> total_time;
            start_time = (int)(total_time - start_time * 1.0 / CLK_TCK);
            // qDebug() << pid << "\t" << start_time << endl;
            QDateTime time =
                QDateTime::fromTime_t(start_time, Qt::UTC, -8 * 60 * 60);
            QString s_time = time.toString("hh:mm:ss");
            model->setItem(i, 3, new QStandardItem(s_time));
            uptime.close();
            model->item(i, 3)->setTextAlignment(Qt::AlignCenter);
          }
        }
        input.close();
        // username

        QString filename2 = "/proc/";
        filename2 = filename2 + QString(QLatin1String(ptr->d_name));
        std::ifstream input2;
        input2.open((filename2 + "/status").toLatin1().data());

        if (input2.is_open()) {
          int j = 0;
          char buffer[100];
          while (j < 7) {
            j++;
            input2.getline(buffer, 100);
          }
          std::string s_uid;
          int uid;
          input2 >> s_uid >> uid;
          if (username_dict->find(uid) != username_dict->end()) {
            std::string user = (*username_dict)[uid];
            model->setItem(i, 5, new QStandardItem(user.c_str()));
          } else {
            model->setItem(i, 5, new QStandardItem("nobody"));
          }
          model->item(i, 5)->setTextAlignment(Qt::AlignLeft);

          if (this->MemState == true) {
            j = 0;
            while (j < 9) {
              j++;
              input2.getline(buffer, 100);
            }

            std::string tmp;
            int vmsize, vmrss;
            input2 >> tmp >> vmsize;
            input2.getline(buffer, 100);
            input2.getline(buffer, 100);
            input2.getline(buffer, 100);
            input2.getline(buffer, 100);
            input2 >> tmp >> vmrss;
            float rate = vmrss * 100.0 / this->mem_size;
            QString temp = tr("%1%").arg(QString::number(rate, 'f', 2));
            if (temp.length() < 6) {
              temp = tr("0%1").arg(temp);
            }
            model->setItem(i, 4, new QStandardItem(temp));
            model->item(i, 4)->setTextAlignment(Qt::AlignCenter);
          }
          input2.close();
        } else {
          qDebug() << tr("Cannot open status: %1!").arg(ptr->d_name) << endl;
        }
      } else {
        // qDebug() << tr("Cannot open %1!").arg(ptr->d_name) << endl;
        i--;
      }

      i++;
    }
  }

  model->sort(2, Qt::DescendingOrder);
}

void NodeInfo::ReFocus() {
  //  ros::V_string nodes;
  //  ros::master::getNodes(nodes);
  //  for (auto i : nodes) {
  //    cout << i << endl;
  //  }

  //  for (int i = 0; i < nodes.size(); i++) {
  //    if (!nodePIDs.contains(GetProcessPIDbyName(
  //            QString::fromUtf8(nodes.at(i).c_str()).mid(1).toLatin1().data())))
  //      nodePIDs.append(GetProcessPIDbyName(
  //          QString::fromUtf8(nodes.at(i).c_str()).mid(1).toLatin1().data()));
  //  }
  nodePIDs = GetAllNodePid();

  if (nodePIDs.size() < model->rowCount())
    model->removeRow(model->rowCount() - 1);

  if (last_pid == -1) {
    return;
  }
  for (int i = 0; i < model->rowCount(); i++) {
    if (model->item(i, 0)->text().toInt() == last_pid) {
      ui->node_info_tableView->setCurrentIndex(model->index(i, 1));
    }
  }
}

bool NodeInfo::isDigit(char *str) {
  int n = strlen(str);
  for (int i = 0; i < n; i++) {
    if (str[i] < '0' || str[i] > '9') {
      return false;
    }
  }
  return true;
}

std::map<int, std::string> *NodeInfo::GetUsernameDict() {
  ifstream input;
  input.open("/etc/passwd");
  map<int, string> *dict;
  if (input.is_open()) {
    dict = new map<int, string>;
    char buffer[100];

    while (input.getline(buffer, 100)) {
      int i = 0;
      char *p = strtok(buffer, ":");
      char *name = NULL;
      while (p && i < 3) {
        if (i == 0) {
          name = p;
        } else if (i == 2) {
          int uid = atoi(p);
          (*dict)[uid] = string(name);
        }
        // qDebug() << p << endl;
        i++;
        p = strtok(NULL, ":");
      }
    }
  } else {
    return NULL;
  }
  // map<int,string>::iterator it;
  // for(it = dict->begin(); it != dict->end(); ++it)
  //     qDebug() <<"key: "<< it->first <<" value: "<< (it->second).c_str() <<
  //     endl;
  return dict;
}

string NodeInfo::GetProperUnit(int num) {
  string str = "KB";
  float fnum = num;
  if (fnum > 1024) {
    fnum /= 1024;
    str = "MB";
  }
  if (fnum > 1024) {
    fnum /= 1024;
    str = "GB";
  }
  char buf[20];
  if (fnum > 99) {
    sprintf(buf, "%.1f %s", fnum, str.c_str());
  } else {
    sprintf(buf, "%.2f %s", fnum, str.c_str());
  }

  return string(buf);
}

// slots

void NodeInfo::UpdateStat() {
  if (this->CpuState)
    GetCPUStat();
  if (this->MemState)
    GetMEMStat();

  ReFocus();
}

void NodeInfo::button() {
  this->MemState = !this->MemState;
  this->CpuState = !this->CpuState;
}
} // namespace qt_rviz_demo
