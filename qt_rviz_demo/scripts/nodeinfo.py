#!/usr/bin/env python
# _*_ coding:utf-8 _*_

from __future__ import print_function

import os
import errno
import sys
import socket
import time
import re
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

try: #py3k
    import urllib.parse as urlparse
except ImportError:
    import urlparse

from optparse import OptionParser
import rosnode
import rosgraph
import rosgraph.names
import rostopic
from node_info import NodeInfo

NAME='rosnode'
ID = '/rosnode'
# The string is defined in clients/rospy/src/rospy/impl/tcpros_base.py TCPROSTransport.get_transport_info
CONNECTION_PATTERN = re.compile(r'\w+ connection on port (\d+) to \[(.*) on socket (\d+)\]')

class ROSNodeException(Exception):
    """
    rosnode base exception type
    """
    pass
class ROSNodeIOException(ROSNodeException):
    """
    Exceptions for communication-related (i/o) errors, generally due to Master or Node network communication issues.
    """
    pass

# need for calling node APIs
def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val

_caller_apis = {}
def get_api_uri(master, caller_id, skip_cache=False):
    """
    @param master: rosgraph Master instance
    @type  master: rosgraph.Master
    @param caller_id: node name
    @type  caller_id: str
    @param skip_cache: flag to skip cached data and force to lookup node from master
    @type  skip_cache: bool
    @return: xmlrpc URI of caller_id
    @rtype: str
    @raise ROSNodeIOException: if unable to communicate with master
    """
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api or skip_cache:
        try:
            caller_api = master.lookupNode(caller_id)
            _caller_apis[caller_id] = caller_api
        except rosgraph.MasterError:
            return None
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
    return caller_api

def lookup_uri(master, system_state, topic, uri):
    for l in system_state[0:2]:
        for entry in l:
            if entry[0] == topic:
                for n in entry[1]:
                    if rostopic.get_api(master, n) == uri:
                        return '%s (%s)' % (n, uri)
    return uri

def get_node_names(namespace=None):
    """
    @param namespace: optional namespace to scope return values by. Namespace must already be resolved.
    @type  namespace: str
    @return: list of node caller IDs
    @rtype: [str]
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosgraph.Master(ID)
    try:
        state = master.getSystemState()
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    nodes = []
    if namespace:
        # canonicalize namespace with leading/trailing slash
        g_ns = rosgraph.names.make_global_ns(namespace)
        for s in state:
            for t, l in s:
                nodes.extend([n for n in l if n.startswith(g_ns) or n == namespace])
    else:
        for s in state:
            for t, l in s:
                nodes.extend(l)
    return list(set(nodes))

def get_machines_by_nodes():
    """
    Find machines connected to nodes. This is a very costly procedure as it
    must do N lookups with the Master, where N is the number of nodes.

    @return: list of machines connected
    @rtype: [str]
    @raise ROSNodeIOException: if unable to communicate with master
    """

    master = rosgraph.Master(ID)

    # get all the node names, lookup their uris, parse the hostname
    # from the uris, and then compare the resolved hostname against
    # the requested machine name.
    machines = []
    node_names = get_node_names()
    for n in node_names:
        try:
            uri = master.lookupNode(n)
            h = urlparse.urlparse(uri).hostname
            if h not in machines:
                machines.append(h)

        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
        except rosgraph.MasterError:
            # it's possible that the state changes as we are doing lookups. this is a soft-fail
            continue
    return machines


def get_nodes_by_machine(machine):
    """
    Find nodes by machine name. This is a very costly procedure as it
    must do N lookups with the Master, where N is the number of nodes.

    @return: list of nodes on the specified machine
    @rtype: [str]
    @raise ROSNodeException: if machine name cannot be resolved to an address
    @raise ROSNodeIOException: if unable to communicate with master
    """

    master = rosgraph.Master(ID)
    try:
        machine_actual = [host[4][0] for host in socket.getaddrinfo(machine, 0, 0, 0, socket.SOL_TCP)]
    except:
        raise ROSNodeException("cannot resolve machine name [%s] to address"%machine)

    # get all the node names, lookup their uris, parse the hostname
    # from the uris, and then compare the resolved hostname against
    # the requested machine name.
    matches = [machine] + machine_actual
    not_matches = [] # cache lookups
    node_names = get_node_names()
    retval = []
    for n in node_names:
        try:
            try:
                uri = master.lookupNode(n)
            except rosgraph.MasterError:
                # it's possible that the state changes as we are doing lookups. this is a soft-fail
                continue

            h = urlparse.urlparse(uri).hostname
            if h in matches:
                retval.append(n)
            elif h in not_matches:
                continue
            else:
                r = [host[4][0] for host in socket.getaddrinfo(h, 0, 0, 0, socket.SOL_TCP)]
                if set(r) & set(machine_actual):
                    matches.append(r)
                    retval.append(n)
                else:
                    not_matches.append(r)
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
    return retval

def kill_nodes(node_names):
    """
    Call shutdown on the specified nodes

    @return: list of nodes that shutdown was called on successfully and list of failures
    @rtype: ([str], [str])
    """
    master = rosgraph.Master(ID)

    success = []
    fail = []
    tocall = []
    try:
        # lookup all nodes keeping track of lookup failures for return value
        for n in node_names:
            try:
                uri = master.lookupNode(n)
                tocall.append([n, uri])
            except:
                fail.append(n)
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    for n, uri in tocall:
        # the shutdown call can sometimes fail to succeed if the node
        # tears down during the request handling, so we assume success
        try:
            p = ServerProxy(uri)
            _succeed(p.shutdown(ID, 'user request'))
        except:
            pass
        success.append(n)

    return success, fail

def _sub_rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    """
    Subroutine for rosnode_listnodes(). Composes list of strings to print to screen.

    @param namespace: (default None) namespace to scope list to.
    @type  namespace: str
    @param list_uri: (default False) return uris of nodes instead of names.
    @type  list_uri: bool
    @param list_all: (default False) return names and uris of nodes as combined strings
    @type  list_all: bool
    @return: new-line separated string containing list of all nodes
    @rtype: str
    """
    master = rosgraph.Master(ID)
    nodes = get_node_names(namespace)
    nodes.sort()
    if list_all:
        return '\n'.join(["%s \t%s"%(get_api_uri(master, n) or 'unknown address', n) for n in nodes])
    elif list_uri:
        return '\n'.join([(get_api_uri(master, n) or 'unknown address') for n in nodes])
    else:
        return '\n'.join(nodes)

def rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    """
    Print list of all ROS nodes to screen.

    @param namespace: namespace to scope list to
    @type  namespace: str
    @param list_uri: print uris of nodes instead of names
    @type  list_uri: bool
    @param list_all: print node names and uris
    @param list_all: bool
    """
    print(_sub_rosnode_listnodes(namespace=namespace, list_uri=list_uri, list_all=list_all))

def get_node_info_description(node_name):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master(ID)

    # go through the master system state first
    try:
        state = master.getSystemState()
        pub_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    pubs = sorted([t for t, l in state[0] if node_name in l])
    subs = sorted([t for t, l in state[1] if node_name in l])
    srvs = sorted([t for t, l in state[2] if node_name in l])

    buff = "Node [%s]"%node_name
    if pubs:
        buff += "\nPublications: \n"
        buff += '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in pubs]) + '\n'
    else:
        buff += "\nPublications: None\n"
    if subs:
        buff += "\nSubscriptions: \n"
        buff += '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in subs]) + '\n'
    else:
        buff += "\nSubscriptions: None\n"
    if srvs:
        buff += "\nServices: \n"
        buff += '\n'.join([" * %s"%l for l in srvs]) + '\n'
    else:
        buff += "\nServices: None\n"

    return buff

def get_node_connection_info_description(node_api, master):
    #turn down timeout on socket library
    socket.setdefaulttimeout(5.0)
    node = ServerProxy(node_api)
    system_state = master.getSystemState()

    try:
        pid = _succeed(node.getPid(ID))
        buff = "Pid: %s\n"%pid
        #master_uri = _succeed(node.getMasterUri(ID))
        businfo = _succeed(node.getBusInfo(ID))
        if businfo:
            buff += "Connections:\n"
            for info in businfo:
                dest_id   = info[1]
                direction = info[2]
                transport = info[3]
                topic     = info[4]
                if len(info) > 5:
                    connected = info[5]
                else:
                    connected = True #backwards compatibility

                if connected:
                    buff += " * topic: %s\n"%topic

                    # older ros publisher implementations don't report a URI
                    buff += "    * to: %s\n"%lookup_uri(master, system_state, topic, dest_id)
                    if direction == 'i':
                        buff += "    * direction: inbound"
                    elif direction == 'o':
                        buff += "    * direction: outbound"
                    else:
                        buff += "    * direction: unknown"
                    if len(info) > 6:
                        match = CONNECTION_PATTERN.match(info[6])
                        if match is not None:
                            buff += " (%s - %s) [%s]" % match.groups()
                    buff += "\n"
                    buff += "    * transport: %s\n"%transport
    except socket.error:
        raise ROSNodeIOException("Communication with node[%s] failed!"%(node_api))
    return buff

def rosnode_info(node_name, quiet=False):
    """
    Print information about node, including subscriptions and other debugging information. This will query the node over the network.

    @param node_name: name of ROS node
    @type  node_name: str
    @raise ROSNodeIOException: if unable to communicate with master
    """
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master(ID)
    node_name = rosgraph.names.script_resolve_name('rosnode', node_name)

    print('-'*80)
    print(get_node_info_description(node_name))

    node_api = get_api_uri(master, node_name)
    if not node_api:
        print("cannot contact [%s]: unknown node"%node_name, file=sys.stderr)
        return

    if not quiet:
        print("\ncontacting node %s ..." % node_api)
        print(get_node_connection_info_description(node_api, master))

# backwards compatibility (deprecated)
rosnode_debugnode = rosnode_info

def get_pid_by_name(node_name):
    master = rosgraph.Master(ID)
    node_api = get_api_uri(master, node_name)
    node = ServerProxy(node_api)
    pid = _succeed(node.getPid(ID))
    print(pid)


if __name__ == '__main__':
#    nodeinfo = NodeInfo()
#    nodeinfo.get_all_node_info()
    for node_name in rosnode.get_node_names():
        get_pid_by_name(node_name)
