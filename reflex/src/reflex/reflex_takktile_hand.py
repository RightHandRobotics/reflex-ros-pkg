#!/usr/bin/env python

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
from std_srvs.srv import Empty

import reflex_msgs.msg
import reflex_msgs.srv
import finger
from reflex_hand import ReflexHand
from reflex_takktile_motor import ReflexTakktileMotor


class ReflexTakktileHand(ReflexHand):
    def __init__(self, prefix, num):
        if (prefix == 'hand1'):
            self.prefix = '/hand1'
            super(ReflexTakktileHand, self).__init__('/reflex_takktile', self.prefix, num, ReflexTakktileMotor)
        else:
            self.prefix = ''
            super(ReflexTakktileHand, self).__init__('/' + prefix + '/reflex_takktile', self.prefix, num, ReflexTakktileMotor)

        self.base_name = '/reflex_takktile'
        self.motor_cmd_pub = rospy.Publisher(self.namespace + '/radian_hand_command',
                                             reflex_msgs.msg.RadianServoCommands, queue_size=10)
        self.fingers = {self.namespace + '_f1': finger.Finger(),
                        self.namespace + '_f2': finger.Finger(),
                        self.namespace + '_f3': finger.Finger()}
        self._connect_motors_to_fingers()
        self.set_speed_service = rospy.ServiceProxy(self.namespace + '/set_speed', reflex_msgs.srv.SetSpeed)
        self.calibrate_fingers_service = rospy.ServiceProxy(self.namespace + '/calibrate_fingers', Empty)
        self.calibrate_tactile_service = rospy.ServiceProxy(self.namespace + '/calibrate_tactile', Empty)
        rospy.Service(self.namespace + '/enable_tactile_stops', Empty, self.enable_tactile_stops)
        rospy.Service(self.namespace + '/disable_tactile_stops', Empty, self.disable_tactile_stops)
        self.tactile_stops_enabled = False
        self.comms_timeout = 5.0  # Seconds with no communications until hand stops
        self.latest_update = rospy.get_rostime()
        rospy.Subscriber(self.namespace + '/hand_state',
                         reflex_msgs.msg.Hand, self._receive_hand_state_cb)

    def _receive_cmd_cb(self, data):
        reset = self.tactile_stops_enabled
        if reset:
            self.disable_tactile_stops()
        self.disable_force_control()
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)
        if reset:
            self.enable_tactile_stops()

    def _receive_angle_cmd_cb(self, data):
        reset = self.tactile_stops_enabled
        if reset:
            self.disable_tactile_stops()
        self.disable_force_control()
        self.reset_speeds()
        self.set_angles(data)
        if reset:
            self.enable_tactile_stops()

    def _receive_vel_cmd_cb(self, data):
        reset = self.tactile_stops_enabled
        if reset:
            self.disable_tactile_stops()
        self.disable_force_control()
        self.set_velocities(data)
        if reset:
            self.enable_tactile_stops()

    def _receive_force_cmd_cb(self, data):
        self.disable_tactile_stops()
        self.disable_force_control()
        self.reset_speeds()
        self.set_force_cmds(data)
        self.enable_force_control()

    def _connect_motors_to_fingers(self):
        for ID, motor in self.motors.items():
            if ID != self.namespace + '_preshape':
                motor.finger = self.fingers[ID]

    def _receive_hand_state_cb(self, data):
        # rospy.loginfo("Calling %s receive_hand_state_cb", self.prefix);
        self.latest_update = rospy.get_rostime()
        self.motors[self.namespace + '_f1']._receive_state_cb(data.motor[0])
        self.motors[self.namespace + '_f2']._receive_state_cb(data.motor[1])
        self.motors[self.namespace + '_f3']._receive_state_cb(data.motor[2])
        self.motors[self.namespace + '_preshape']._receive_state_cb(data.motor[3])
        self.fingers[self.namespace + '_f1']._receive_state_cb(data.finger[0])
        self.fingers[self.namespace + '_f2']._receive_state_cb(data.finger[1])
        self.fingers[self.namespace + '_f3']._receive_state_cb(data.finger[2])

    def disable_tactile_stops(self, data=None):
        self.tactile_stops_enabled = False
        for ID, motor in self.motors.items():
            motor.disable_tactile_stops()
        rospy.sleep(0.05)  # Lets commands stop before allowing any other actions
        return []

    def enable_tactile_stops(self, data=None):
        rospy.sleep(0.07)  # Lets other actions happen before beginning constant commands
        self.tactile_stops_enabled = True
        for ID, motor in self.motors.items():
            motor.enable_tactile_stops()
        return []

    def calibrate_fingers(self):
        self.calibrate_fingers_service()

    def calibrate_tactile(self):
        self.calibrate_tactile_service()

    def _publish_motor_commands(self):
        '''
        Checks whether motors have updated their states and, if so, publishes
        their commands
        '''
        speed_update_occurred = False
        for ID, motor in self.motors.items():
            speed_update_occurred |= motor.speed_update_occurred
        position_update_occurred = False
        for ID, motor in self.motors.items():
            position_update_occurred |= motor.position_update_occurred
        if speed_update_occurred:
            self._publish_speed_commands()
        if position_update_occurred:
            self._publish_position_commands()

    def _publish_speed_commands(self):
        '''
        Queries the motors for their speed and position setpoints and publishes
        those to the appropriate topics for reflex_driver
        '''
        for ID, motor in self.motors.items():
            motor.speed_update_occurred = False
        motor_speed_cmd = reflex_msgs.srv.SetSpeedRequest(
            [self.motors[self.namespace + '_f1'].get_commanded_speed(),
             self.motors[self.namespace + '_f2'].get_commanded_speed(),
             self.motors[self.namespace + '_f3'].get_commanded_speed(),
             self.motors[self.namespace + '_preshape'].get_commanded_speed()])
        self.set_speed_service(motor_speed_cmd)

    def _publish_position_commands(self):
        '''
        Queries the motors for their speed and position setpoints and publishes
        those to the appropriate topics for reflex_driver
        '''
        for ID, motor in self.motors.items():
            motor.position_update_occurred = False
        motor_pos_cmd = reflex_msgs.msg.RadianServoCommands(
            [self.motors[self.namespace + '_f1'].get_commanded_position(),
             self.motors[self.namespace + '_f2'].get_commanded_position(),
             self.motors[self.namespace + '_f3'].get_commanded_position(),
             self.motors[self.namespace + '_preshape'].get_commanded_position()])
        self.motor_cmd_pub.publish(motor_pos_cmd)
        rospy.sleep(0.01)