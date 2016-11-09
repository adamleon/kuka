#!/usr/bin/env python


# Copyright (c) 2014, Norwegian University of Science and Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Lars Tingelstad
# Maintainer: Lars Tingelstad <lars.tingelstad@ntnu.no>

import threading
import socket
import struct
import numpy as np

import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

class RobotSensorInterfaceConnection(object):

    def __init__(self, host, port):
        self.__thread = None
        self.__socket = None
        self.__keep_running = False

        self._lock = threading.Lock()

        self._q_actual = None
        self._q_actual_deg = None
        self._q_desired = None
        self._q_desired_deg = None
        self._q_error = None
        self._ipoc = None

        self._host = host
        self._port = port

    def connect(self):
        if self.__socket:
            self.disconnect()
        self.__keep_running = True

        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.__socket.settimeout(0.004)
        self.__socket.sendto("RSI", (self._host, self._port))

        self.__thread = threading.Thread(name='RobotSensorInterfaceConnection', target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()

    def disconnect(self):
        if self.__thread:
            self.__keep_running = False
            self.__thread.join()
            self.__thread = None
        if self.__socket:
            self.__socket.close()
            self.__socket = None

    def __on_packet(self, data):
        with self._lock:
            ## Unpack rsi package
            recv = struct.unpack('Q6f', data)
            ## IPOC timestamp
            self._ipoc = recv[0]
            ## Actual joint positions
            self._q_actual_deg = np.array(recv[1:], dtype=np.float32)
            self._q_actual = np.deg2rad(self._q_actual_deg)

            if self._q_desired is None:
                self._q_desired = self._q_actual
                self._q_desired_deg = self._q_actual_deg

            self._q_error = self._q_actual - self._q_desired


    def set_desired_joint_states(self, q_desired):
        #rospy.loginfo("Setting desired joint states")
        with self._lock:
            self._q_desired = q_desired.copy()
            self._q_desired_deg = np.rad2deg(self._q_desired)

    def get_joint_states(self):
        with self._lock:
            return self._q_actual, self._q_desired, self._q_error

    def __run(self):
        while self.__keep_running:
            data, addr = self.__socket.recvfrom(1024)

            if data:
                self.__on_packet(data)

            with self._lock:
                self.__socket.sendto(struct.pack('Q6f', self._ipoc, *self._q_desired_deg), addr)

if __name__ == '__main__':

    args = rospy.myargv()[1:]

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('robot_ip', type=str, default='',
                        help='Please specify the robot (RSI) hostname')
    parser.add_argument('--robot_port', type=int, default=10000,
                        help='Please specify the robot (RSI) port number')

    args = parser.parse_args(args)
    robot_host = args.robot_ip
    robot_port = args.robot_port

    rospy.init_node('kuka_robot_sensor_interface_connection')
    global joint_names
    prefix = ""
    joint_names = [prefix + name for name in JOINT_NAMES]

    connection = RobotSensorInterfaceConnection(robot_host, robot_port)
    connection.connect()

    try:
        rospy.spin()
    except:
        rospy.ROSInterruptException()
