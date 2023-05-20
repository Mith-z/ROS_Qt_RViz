#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import os
import subprocess
from node_info import NodeInfo

def func():
    node_info = NodeInfo()
    return node_info.func()
#    filename = os.path.realpath(__file__)
#    dir = os.path.dirname(filename)
#    output = subprocess.check_output('python ' + dir + '/' + 'node_info.py',shell = True).decode('utf-8')
#    return output



if __name__ == '__main__':
    func()

