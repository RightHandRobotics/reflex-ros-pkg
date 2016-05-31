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

from motor import Motor


class ReflexTakktileMotor(Motor):
    def __init__(self, name):
        super(ReflexTakktileMotor, self).__init__(name)
        self.motor_cmd = 0.0
        self.speed = 0.0
        self.finger = None
        self.tactile_stops_enabled = False
        self.position_update_occurred = False
        self.speed_update_occurred = False
        self.reset_motor_speed()

    def get_commanded_position(self):
        return self.motor_cmd

    def get_commanded_speed(self):
        return self.speed

    def set_motor_angle(self, goal_pos):
        '''
        Bounds the given position command and sets it to the motor
        '''
        if not nearly_equal(self.motor_cmd, self._check_motor_angle_command(goal_pos), 3) or\
           not nearly_equal(self.motor_cmd, self._motor_msg.joint_angle, 1):
            self.position_update_occurred = True
        self.motor_cmd = self._check_motor_angle_command(goal_pos)

    def _check_motor_angle_command(self, angle_command):
        '''
        Returns given command if within the allowable range,
        returns bounded command if out of range
        '''
        bounded_command = min(max(angle_command, 0.0), self._MAX_MOTOR_TRAVEL)
        return bounded_command

    def set_motor_speed(self, goal_speed):
        '''
        Bounds the given position command and sets it to the motor
        '''
        if not nearly_equal(self.speed, self._check_motor_speed_command(goal_speed), 2):
            self.speed_update_occurred = True
        self.speed = self._check_motor_speed_command(goal_speed)

    def reset_motor_speed(self):
        '''
        Resets speed to default
        '''
        if not nearly_equal(self.speed, self._DEFAULT_MOTOR_SPEED, 2):
            self.speed_update_occurred = True
        self.speed = self._DEFAULT_MOTOR_SPEED

    def set_motor_velocity(self, goal_vel):
        '''
        Sets speed and commands finger in or out based on sign of velocity
        '''
        self.set_motor_speed(goal_vel)
        if goal_vel > 0.0:
            self.set_motor_angle(self._MAX_MOTOR_TRAVEL)
        elif goal_vel <= 0.0:
            self.set_motor_angle(0.0)

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        self.set_motor_angle(self._motor_msg.joint_angle + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        self.set_motor_angle(self._motor_msg.joint_angle - loosen_angle)

    def _receive_state_cb(self, data):
        self._motor_msg = data
        self._handle_motor_load(data.load)

    def _handle_motor_load(self, load):
        if self._in_control_force_mode:
            self._control_force(load, k=16e-4*0.025)
        elif self.finger and self.tactile_stops_enabled:
            self._loosen_if_in_contact()
        self._loosen_if_overloaded(load)

    def _loosen_if_in_contact(self):
        '''
        Takes the finger tactile data, loosens motor if in contact
        '''
        tolerance = 0.001
        if self.finger.is_finger_in_contact() and (self.motor_cmd > self._motor_msg.joint_angle + tolerance):
            rospy.logdebug("Motor %s in contact", self.name)
            self.loosen(0)

    def disable_tactile_stops(self):
        self.tactile_stops_enabled = False

    def enable_tactile_stops(self):
        self.tactile_stops_enabled = True


def nearly_equal(a, b, sig_fig=3):
    return (a == b or int(a*10**sig_fig) == int(b*10**sig_fig))
