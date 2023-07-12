#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import os
import subprocess
import time
import threading
from node_info import NodeInfo

def all_nodes_info():
    node_info = NodeInfo()
    return node_info.get_all_node_infos()
#    filename = os.path.realpath(__file__)
#    dir = os.path.dirname(filename)
#    output = subprocess.check_output('python ' + dir + '/' + 'node_info.py',shell = True).decode('utf-8')
#    return output

def pcl_publish():
    t = threading.Thread(target=publish)
    t.start()

def publish():
    p = subprocess.Popen(["python", "/home/mith/catkin_qt/src/qt_rviz_demo/scripts/pcl_publish.py"])
    p.wait()

if __name__ == '__main__':
    all_nodes_info()

