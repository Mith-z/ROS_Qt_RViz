#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import os
import subprocess

def func():
    filename = os.path.realpath(__file__)
    dir = os.path.dirname(filename)
    output = subprocess.check_output('python ' + dir + '/' + 'nodeinfo.py',shell = True).decode('utf-8')
    return output



if __name__ == '__main__':
    func()

