#!/usr/bin/env python2.7
# _*_ coding:utf-8 _*_

import rosnode
import rospy

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy
from socket import error as SocketError

import psutil
import sys


from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import QApplication

ID = '/NODEINFO'


class NodeInfo(object):
    nodes = dict()
    NODE_FIELDS   = [             'pid', 'get_cpu_percent', 'get_memory_percent', 'get_num_threads']
    FORMAT_STRS   = ['%s',        '%s',  '%0.2f',           '%0.2f',              '%s'             ]
    OUT_FIELDS    = ['node_name', 'pid', 'cpu_percent',     'memory_percent',     'num_threads'    ]

    def get_node_info(self, node_name, skip_cache=False):
        node_api = rosnode.get_api_uri(rospy.get_master(), node_name, skip_cache=skip_cache)
        try:
            code, msg, pid = ServerProxy(node_api[2]).getPid(ID)
            if node_name in self.nodes:
                return self.nodes[node_name]
            else:
                try:
                    p = psutil.Process(pid)
                    self.nodes[node_name] = p
                    return p
                except:
                    return False
        except SocketError:
            if not skip_cache:
                return self.get_node_info(node_name, skip_cache=True)
            else:
                return False

    def get_all_node_info(self):
        infos = []
        self.remove_dead_nodes()
        for node_name in rosnode.get_node_names():
            info = self.get_node_info(node_name)
            if info is not False:
                infos.append((node_name, info))
#        print(infos)
        return infos

    def get_all_node_fields(self, fields):
        processes = self.get_all_node_info()
        infos = []
        psutil_v2_api = int(psutil.__version__.split('.')[0]) >= 2
        for name, p in processes:
            all_fields = fields + ['cmdline', 'get_memory_info']
            if psutil_v2_api:
                all_fields = [
                    f[4:] if f.startswith('get_') else f
                    for f in all_fields]
            infos.append(self.as_dict(p, all_fields))
            infos[-1]['node_name'] = name
        #print(infos)
        return infos

    def remove_dead_nodes(self):
        running_nodes = rosnode.get_node_names()
        dead_nodes = [node_name for node_name in self.nodes if node_name not in running_nodes]
        for node_name in dead_nodes:
            self.nodes.pop(node_name, None)

    def kill_node(self, node_name):
        success, fail = rosnode.kill_nodes([node_name])
        return node_name in success

    def as_dict(self, p, attrs=[], ad_value=None):
        # copied code from psutil.__init__ from a newer version
        excluded_names = set(['send_signal', 'suspend', 'resume', 'terminate',
                              'kill', 'wait', 'is_running', 'as_dict', 'parent',
                              'get_children', 'nice'])
        retdict = dict()
        for name in set(attrs or dir(p)):
            if name.startswith('_'):
                continue
            if name.startswith('set_'):
                continue
            if name in excluded_names:
                continue
            try:
                attr = getattr(p, name)
                if callable(attr):
                    if name == 'get_cpu_percent':
                        ret = attr(interval=0)
                    else:
                        ret = attr()
                else:
                    ret = attr
            except psutil.AccessDenied:
                ret = ad_value
            except NotImplementedError:
                # in case of not implemented functionality (may happen
                # on old or exotic systems) we want to crash only if
                # the user explicitly asked for that particular attr
                if attrs:
                    raise
                continue
            if name.startswith('get'):
                if name[3] == '_':
                    name = name[4:]
                elif name == 'getcwd':
                    name = 'cwd'
            retdict[name] = ret
        return retdict

    #供C++调用的函数
    def get_all_node_infos(self):
        infos = self.get_all_node_fields(self.NODE_FIELDS)
        out = ''
        result = ''
        for nx, info in enumerate(infos):
            for col, field in enumerate(self.OUT_FIELDS):
                val = info[field]
                out = out + self.FORMAT_STRS[col] % val+ ' '
            result = result + out + '\n'
            out = ''
        return result

if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('my_node')

    app = QApplication(sys.argv)
    node = NodeInfo()
    node.func()





