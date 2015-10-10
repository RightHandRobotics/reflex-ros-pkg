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

import reflex_msgs.msg


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_takktile_f1 or /reflex_sf_f1
        '''
        self.name = name[1:]
        self._DEFAULT_MOTOR_SPEED = rospy.get_param(name + '/default_motor_speed')
        self._MAX_MOTOR_SPEED = rospy.get_param(name + '/max_motor_speed')
        self._MAX_MOTOR_TRAVEL = rospy.get_param(name + '/max_motor_travel')
        self._OVERLOAD_THRESHOLD = rospy.get_param(name + '/overload_threshold')
        self._motor_msg = reflex_msgs.msg.Motor()
        self._in_control_force_mode = False

    def get_current_joint_angle(self):
        return self._motor_msg.joint_angle

    def get_load(self):
        return self._motor_msg.load

    def get_motor_msg(self):
        return self._motor_msg

    def set_motor_angle(self, goal_pos):
        raise NotImplementedError

    def _check_motor_angle_command(self, angle_command):
        raise NotImplementedError

    def set_motor_speed(self, goal_speed):
        raise NotImplementedError

    def reset_motor_speed(self):
        raise NotImplementedError

    def set_motor_velocity(self, goal_vel):
        raise NotImplementedError

    def tighten(self, tighten_angle=0.05):
        raise NotImplementedError

    def loosen(self, loosen_angle=0.05):
        raise NotImplementedError

    def _receive_state_cb(self, data):
        raise NotImplementedError

    def _handle_motor_load(self, load):
        raise NotImplementedError

    def _check_motor_speed_command(self, goal_speed):
        '''
        Returns absolute of given command if within the allowable range,
        returns bounded command if out of range. Always returns positive
        '''
        bounded_command = min(abs(goal_speed), self._MAX_MOTOR_SPEED)
        return bounded_command

    def enable_force_control(self):
        self._in_control_force_mode = True
        self.previous_load_control_output = self.get_current_joint_angle()
        self.previous_load_control_error = 0.0

    def disable_force_control(self):
        self._in_control_force_mode = False

    def set_force_cmd(self, force_cmd):
        '''
        Bounds the given goal load and sets it as the goal
        '''
        self.force_cmd = min(max(force_cmd, 0.0), self._OVERLOAD_THRESHOLD)

    def _control_force(self, current_force, k):
        '''
        Uses discrete integral control to try and maintain goal force
        k is Compensator gain - higher gain has faster response and is more unstable
        '''
        current_error = self.force_cmd - current_force
        output = self.previous_load_control_output + k * (current_error + self.previous_load_control_error)
        self.set_motor_angle(output)
        self.previous_load_control_output = output
        self.previous_load_control_error = current_error

    def _loosen_if_overloaded(self, load):
        '''
        Takes the given load and checks against threshold, loosen motor if over
        '''
        if abs(load) > self._OVERLOAD_THRESHOLD:
            rospy.logwarn("Motor %s overloaded at %f, loosening", self.name, load)
            self.loosen()
