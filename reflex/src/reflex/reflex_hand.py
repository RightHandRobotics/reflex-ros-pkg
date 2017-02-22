#############################################################################
# Copyright 2015 Right Hand Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#############################################################################

__author__ = 'Eric Schneider'
__copyright__ = 'Copyright (c) 2015 RightHand Robotics'
__license__ = 'Apache License 2.0'
__maintainer__ = 'RightHand Robotics'
__email__ = 'reflex-support@righthandrobotics.com'


import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty

import reflex_msgs.msg


class ReflexHand(object):
    def __init__(self, name, prefix, num, MotorClass):
        '''
        Assumes that "name" is the name of the hand with a preceding
        slash, e.g. /reflex_takktile or /reflex_sf
        '''
        self.namespace = name
        self.prefix = prefix
        rospy.init_node('reflex_hand_' + num, anonymous=True)
        rospy.loginfo('Starting up hand')
        self.motors = {self.namespace + '_f1': MotorClass(self.prefix + self.namespace + '_f1'),
                       self.namespace + '_f2': MotorClass(self.prefix + self.namespace + '_f2'),
                       self.namespace + '_f3': MotorClass(self.prefix + self.namespace + '_f3'),
                       self.namespace + '_preshape': MotorClass(self.prefix + self.namespace + '_preshape')}
        rospy.Subscriber(self.namespace + '/command',
                         reflex_msgs.msg.Command, self._receive_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_position',
                         reflex_msgs.msg.PoseCommand, self._receive_angle_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_velocity',
                         reflex_msgs.msg.VelocityCommand, self._receive_vel_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_motor_force',
                         reflex_msgs.msg.ForceCommand, self._receive_force_cmd_cb)
        rospy.loginfo('ReFlex hand has started, waiting for commands...')

    def _receive_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_angle_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_vel_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_force_cmd_cb(self, data):
        raise NotImplementedError

    def set_angles(self, pose):
        self.motors[self.namespace + '_f1'].set_motor_angle(pose.f1)
        self.motors[self.namespace + '_f2'].set_motor_angle(pose.f2)
        self.motors[self.namespace + '_f3'].set_motor_angle(pose.f3)
        self.motors[self.namespace + '_preshape'].set_motor_angle(pose.preshape)

    def set_velocities(self, velocity):
        self.motors[self.namespace + '_f1'].set_motor_velocity(velocity.f1)
        self.motors[self.namespace + '_f2'].set_motor_velocity(velocity.f2)
        self.motors[self.namespace + '_f3'].set_motor_velocity(velocity.f3)
        self.motors[self.namespace + '_preshape'].set_motor_velocity(velocity.preshape)

    def set_speeds(self, speed):
        self.motors[self.namespace + '_f1'].set_motor_speed(speed.f1)
        self.motors[self.namespace + '_f2'].set_motor_speed(speed.f2)
        self.motors[self.namespace + '_f3'].set_motor_speed(speed.f3)
        self.motors[self.namespace + '_preshape'].set_motor_speed(speed.preshape)

    def set_force_cmds(self, torque):
        self.motors[self.namespace + '_f1'].set_force_cmd(torque.f1)
        self.motors[self.namespace + '_f2'].set_force_cmd(torque.f2)
        self.motors[self.namespace + '_f3'].set_force_cmd(torque.f3)
        self.motors[self.namespace + '_preshape'].set_force_cmd(torque.preshape)

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def disable_force_control(self):
        for ID, motor in self.motors.items():
            motor.disable_force_control()
        rospy.sleep(0.05)  # Lets commands stop before allowing any other actions

    def enable_force_control(self):
        rospy.sleep(0.05)  # Lets other actions happen before beginning constant torque commands
        for ID, motor in self.motors.items():
            motor.enable_force_control()
