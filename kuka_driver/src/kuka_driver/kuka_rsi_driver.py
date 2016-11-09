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

import time
import threading

import numpy as np

# ROS imports
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kuka_robot_sensor_interface_connection import RobotSensorInterfaceConnection


class KUKADriverNode(object):

    def __init__(self, host, port):
        self._host = host
        self._port = port

        self._lock = threading.Lock()

        # Publish rate (Hz) - Set to half of the rsi control rate
        self._pub_rate = rospy.get_param('pub_rate', 125)
        rospy.loginfo("Setting publish rate (hz) based on parameter: %f", self._pub_rate)

        # Joint names
        def_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self._joint_names = rospy.get_param('controller_joint_names', def_joint_names)
        if len(self._joint_names) == 0:
            rospy.logwarn("Joint list is empty, did you set controller_joint_name?")
            rospy.loginfo("Setting joint names based on parameter: %s", str(self.joint_names))

        # RSI connection
        self._rsi_connection = RobotSensorInterfaceConnection(host, port)

        # Publishers
        self._joint_state_pub = rospy.Publisher('joint_states',
                                                JointState)
        self._joint_feedback_pub = rospy.Publisher('feedback_states',
                                                   FollowJointTrajectoryFeedback)
        # Subscribers
        self._joint_trajectory_sub = rospy.Subscriber('joint_path_command',
                                                      JointTrajectory,
                                                      self._on_trajectory)
        self._joint_trajectory_point_sub = rospy.Subscriber('joint_command',
                                                            JointTrajectoryPoint,
                                                            self._on_trajectory_point)

        # Timed task (started automatically)
        period = rospy.Duration(1.0/self._pub_rate)
        rospy.loginfo('Setting up publish worker with period (sec): %s', str(period.to_sec()))
        rospy.Timer(period, self._publish_worker)

    def start(self):
        rospy.loginfo('Starting kuka_driver')
        self._rsi_connection.connect()

    def _on_trajectory(self, msg):
        try:
            rospy.loginfo('Received trajectory with %s points, executing callback', str(len(msg.points)))

            points = msg.points
            for i in range(len(points) - 1):
                self._rsi_connection.set_desired_joint_states(np.array(points[i].positions))
                time.sleep(points[i+1].time_from_start.to_sec() - points[i].time_from_start.to_sec())

            self._rsi_connection.set_desired_joint_states(np.array(points[-1].positions))

        except Exception as e:
            rospy.logerr('Unexpected exception: %s', e)

    def _on_trajectory_point(self, msg):
        try:
            #rospy.loginfo('Received trajectory point, executing callback')
            joint_position_command = np.array(msg.positions)
            self._rsi_connection.set_desired_joint_states(joint_position_command)
        except Exception as e:
            rospy.logerr('Unexpected exception: %s', e)


    def _publish_worker(self, event):
        self._state_publisher()

    def _state_publisher(self):
        try:
            joint_state_msg = JointState()
            joint_fb_msg = FollowJointTrajectoryFeedback()

            with self._lock:

                time = rospy.Time.now()

                q_actual, q_desired, q_error = self._rsi_connection.get_joint_states()

                #Joint states
                joint_state_msg.header.stamp = time
                joint_state_msg.name = self._joint_names
                joint_state_msg.position = q_actual

                self._joint_state_pub.publish(joint_state_msg)

                #Joint feedback
                joint_fb_msg.header.stamp = time
                joint_fb_msg.joint_names = self._joint_names
                joint_fb_msg.actual.positions = q_actual
                joint_fb_msg.desired.positions = q_desired
                joint_fb_msg.error.positions = q_error

                self._joint_feedback_pub.publish(joint_fb_msg)

        except Exception as e:
            rospy.logerr('Unexpected exception in joint state publisher: %s', e)

def main():
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

    rospy.init_node('kuka_driver')

    driver = KUKADriverNode(robot_host, robot_port)
    driver.start()


    try:
        rospy.spin()
    except:
        rospy.ROSInterruptException()



if __name__ == '__main__':
    main()
