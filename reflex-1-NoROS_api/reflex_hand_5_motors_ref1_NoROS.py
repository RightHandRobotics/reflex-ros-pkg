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

__author__ = 'Michael Kaca'
__copyright__ = 'Copyright (c) 2015 RightHand Robotics'
__license__ = 'Apache License 2.0'
__maintainer__ = 'RightHand Robotics'
__email__ = 'reflex-support@righthandrobotics.com'


import sys
sys.path.insert(0, './dynamixel_motor/dynamixel_driver/src/dynamixel_driver/')
import dynamixel_io


from reflex_one_motor_NoROS import ReflexOneMotor
from reflex_motor_information import reflex_motor_information
from time import sleep


class ReflexHand(object):
    def __init__(self, name, MotorClass):
        '''
        Assumes that "name" is the name of the hand with a preceding
        slash, e.g. /reflex_takktile or /reflex_sf
        '''
        self.namespace = name

        self.dxl_io = dynamixel_io.DynamixelIO("/dev/ttyUSB0", 1000000)

        print 'Starting up the hand'
        r_m_i = reflex_motor_information
        motor_names = ['_f1','_f2', '_f3','_preshape1','_preshape2']
        self.motors = {}
        for motor_name in motor_names:
            motorname = self.namespace + motor_name
            self.motors[motorname] = ReflexOneMotor(r_m_i[motorname]["name"],
            r_m_i[motorname]["ID"],
            r_m_i[motorname]["_DEFAULT_MOTOR_SPEED"],
            r_m_i[motorname]["_MAX_MOTOR_SPEED"],
            r_m_i[motorname]["_MAX_MOTOR_TRAVEL"],
            r_m_i[motorname]["MOTOR_TO_JOINT_INVERTED"],
            r_m_i[motorname]["_OVERLOAD_THRESHOLD"],
            r_m_i[motorname]["MOTOR_TO_JOINT_GEAR_RATIO"],
            r_m_i[motorname]["MODEL"],
            r_m_i[motorname]["PROTOCOL"],
            )
        #self.namespace + '_f2':ReflexOneMotor(self.namespace + '_f2',2,0.5,6.1,3.0,1,0.25,1.42,12,1)}
                    #   self.namespace + '_f2': MotorClass(self.namespace + '_f2',4.5,3.75,0.25),
                    #   self.namespace + '_f3': MotorClass(self.namespace + '_f3',4.5,3.75,0.25),
                    #   self.namespace + '_preshape1': MotorClass(self.namespace + '_preshape1',4.5,6,0.55),
                    #   self.namespace + '_preshape2': MotorClass(self.namespace + '_preshape2',4.5,6,0.55)}


#rospy.Subscriber(self.namespace + '/command_motor_force',
#                 reflex_msgs.msg.ForceCommand, self._receive_force_cmd_cb)
        print 'ReFlex hand has started, waiting for commands...'

    def _receive_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_angle_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_vel_cmd_cb(self, data):
        raise NotImplementedError

    def _receive_force_cmd_cb(self, data):
        raise NotImplementedError

    def set_angles(self, pose):

        self.motors[self.namespace + '_f1'].set_motor_angle(pose.get('f1', 0), self.dxl_io)
        self.motors[self.namespace + '_f2'].set_motor_angle(pose.get('f2', 0), self.dxl_io)
        self.motors[self.namespace + '_f3'].set_motor_angle(pose.get('f3', 0), self.dxl_io)
        self.motors[self.namespace + '_preshape1'].set_motor_angle(pose.get('preshape1', 0), self.dxl_io)
        self.motors[self.namespace + '_preshape2'].set_motor_angle(pose.get('preshape2', 0), self.dxl_io)

    def set_velocities(self, velocity):
        self.motors[self.namespace + '_f1'].set_motor_velocity(velocity.get('f1', 0), self.dxl_io)
        self.motors[self.namespace + '_f2'].set_motor_velocity(velocity.get('f2', 0), self.dxl_io)
        self.motors[self.namespace + '_f3'].set_motor_velocity(velocity.get('f3', 0),self.dxl_io)
        self.motors[self.namespace + '_preshape1'].set_motor_velocity(velocity.get('preshape1', 0),self.dxl_io)
        self.motors[self.namespace + '_preshape2'].set_motor_velocity(velocity.get('preshape2', 0),self.dxl_io)

    def set_speeds(self, speed):
        self.motors[self.namespace + '_f1'].set_motor_speed(speed.get('f1', 0), self.dxl_io)
        self.motors[self.namespace + '_f2'].set_motor_speed(speed.get('f2', 0), self.dxl_io)
        self.motors[self.namespace + '_f3'].set_motor_speed(speed.get('f3', 0), self.dxl_io)
        self.motors[self.namespace + '_preshape1'].set_motor_speed(speed.get('preshape1', 0), self.dxl_io)
        self.motors[self.namespace + '_preshape2'].set_motor_speed(speed.get('preshape2', 0), self.dxl_io)

    def set_force_cmds(self, torque):
        self.motors[self.namespace + '_f1'].set_force_cmd(torque.get('f1', 0),self.dxl_io)
        self.motors[self.namespace + '_f2'].set_force_cmd(torque.get('f2', 0),self.dxl_io)
        self.motors[self.namespace + '_f3'].set_force_cmd(torque.get('f3', 0), self.dxl_io)
        self.motors[self.namespace + '_preshape1'].set_force_cmd(torque.get('preshape1', 0), self.dxl_io)
        self.motors[self.namespace + '_preshape2'].set_force_cmd(torque.get('preshape2', 0), self.dxl_io)

    def reset_speeds(self):
        for motor in sorted(self.motors):
            self.motors[motor].reset_motor_speed(self.dxl_io)

    def disable_force_control(self):
        for ID, motor in self.motors.items():
            motor.disable_force_control()
        sleep(0.05)  # Lets commands stop before allowing any other actions

    def enable_force_control(self):
        sleep(0.05)  # Lets other actions happen before beginning constant torque commands
        for ID, motor in self.motors.items():
            motor.enable_force_control()
