#! /usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os


class ScreenHandlerException(Exception):
    pass


class ScreenHandler(object):
    '''
    The class to handle the running screen sessions and create new sessions on
    start of the ROS nodes.
    '''

    LOG_PATH = ''.join([os.environ.get('ROS_LOG_DIR'), os.path.sep]) if os.environ.get('ROS_LOG_DIR') else os.path.join(os.path.expanduser('~'), '.ros/log/')
    SCREEN = "/usr/bin/screen"
    SLASH_SEP = '_'

    @classmethod
    def createSessionName(cls, node=None):
        '''
        Creates a name for the screen session. All slash separators are replaced by
        L{SLASH_SEP}
        @param node: the name of the node
        @type node: C{str}
        @return: name for the screen session.
        @rtype: C{str}
        '''
#    package_name = str(package) if not package is None else ''
#    lanchfile_name = str(launchfile).replace('.launch', '') if not launchfile is None else ''
        node_name = str(node).replace('/', cls.SLASH_SEP) if node is not None else ''
#    result = ''.join([node_name, '.', package_name, '.', lanchfile_name])
        return node_name

    @classmethod
    def splitSessionName(cls, session):
        '''
        Splits the screen session name into PID and session name generated by
        L{createSessionName()}.
        @param session: the screen session name
        @type session: C{str}
        @return: PID, session name generated by L{createSessionName()}. Not presented
          values are coded as empty strings. Not valid session names have an empty
          PID string.
        @rtype: C{str, str}
        '''
        result = session.split('.', 1)
        if len(result) != 2:
            return '', ''
        pid = result[0]
        node = result[1]  # .replace(cls.SLASH_SEP, '/')
#    package = result[2]
#    launch = ''.join([result[3], '.launch']) if len(result[2]) > 0 else result[2]
        return pid, node  # , package, launch

    @classmethod
    def testScreen(cls):
        '''
        Tests for whether the SCREEN binary exists and raise an exception if not.
        @raise ScreenHandlerException: if the screen binary not found.
        '''
        if not os.path.isfile(cls.SCREEN):
            raise ScreenHandlerException(''.join([cls.SCREEN, " is missing"]))

    @classmethod
    def getScreenLogFile(cls, session=None, node=None):
        '''
        Generates a log file name of the ROS log.
        @param node: the name of the node
        @type node: C{str}
        @return: the ROS log file name
        @rtype: C{str}
        @todo: get the run_id from the ROS parameter server and search in this log folder
        for the log file (handle the node started using a launch file).
        '''
        if session is not None:
            return "%s%s.log" % (cls.LOG_PATH, session)
        elif node is not None:
            return "%s%s.log" % (cls.LOG_PATH, cls.createSessionName(node))
        else:
            return "%s%s.log" % (cls.LOG_PATH, 'unknown')

    @classmethod
    def getROSLogFile(cls, node):
        '''
        Generates a log file name for the ROS log
        @param node: the name of the node
        @type node: C{str}
        @return: the log file name
        @rtype: C{str}
        '''
        if node is not None:
            return "%s%s.log" % (cls.LOG_PATH, node.strip('/').replace('/', '_'))
        else:
            return ''

    @classmethod
    def getScreenCfgFile(cls, session=None, node=None):
        '''
        Generates a configuration file name for the screen session.
        @param session: the name of the screen session
        @type session: C{str}
        @return: the configuration file name
        @rtype: C{str}
        '''
        if session is not None:
            return "%s%s.conf" % (cls.LOG_PATH, session)
        elif node is not None:
            return "%s%s.log" % (cls.LOG_PATH, cls.createSessionName(node))
        else:
            return "%s%s.log" % (cls.LOG_PATH, 'unknown')

    @classmethod
    def getScreenPidFile(cls, session=None, node=None):
        '''
        Generates a PID file name for the screen session.
        @param session: the name of the screen session
        @type session: C{str}
        @return: the PID file name
        @rtype: C{str}
        '''
        if session is not None:
            return "%s%s.pid" % (cls.LOG_PATH, session)
        elif node is not None:
            return "%s%s.pid" % (cls.LOG_PATH, cls.createSessionName(node))
        else:
            return "%s%s.pid" % (cls.LOG_PATH, 'unknown')

    @classmethod
    def getSceenCmd(cls, node):
        '''
        Generates a configuration file and return the command prefix to start the given node
        in a screen terminal.
        @param node: the name of the node
        @type node: C{str}
        @return: the command prefix
        @rtype: C{str}
        '''
        filename = cls.getScreenCfgFile(node=node)
        f = None
        try:
            f = open(cls.getScreenCfgFile(node=node), 'w')
        except Exception:
            os.makedirs(os.path.dirname(filename))
            f = open(cls.getScreenCfgFile(node=node), 'w')
        f.write(''.join(["logfile ", cls.getScreenLogFile(node=node), "\n"]))
        f.write("logfile flush 0\n")
        f.write("defscrollback 10000\n")
        ld_library_path = os.getenv('LD_LIBRARY_PATH', '')
        if ld_library_path:
            f.write(' '.join(['setenv', 'LD_LIBRARY_PATH', ld_library_path, "\n"]))
        ros_etc_dir = os.getenv('ROS_ETC_DIR', '')
        if ros_etc_dir:
            f.write(' '.join(['setenv', 'ROS_ETC_DIR', ros_etc_dir, "\n"]))
        f.close()
        return ' '.join([cls.SCREEN, '-c', cls.getScreenCfgFile(node=node), '-L', '-dmS', cls.createSessionName(node=node)])
