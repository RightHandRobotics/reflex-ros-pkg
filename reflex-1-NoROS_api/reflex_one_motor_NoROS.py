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

__author__ = 'Michael Kaca, Harrison Young'
__copyright__ = 'Copyright (c) 2018 RightHand Robotics'
__license__ = 'Apache License 2.0'
__maintainer__ = 'RightHand Robotics'
__email__ = 'reflex-support@righthandrobotics.com'


#from dynamixel_msgs.msg import JointState
#from dynamixel_controllers.srv import TorqueEnable
#from dynamixel_controllers.srv import SetSpeed
import sys
sys.path.insert(0, './dynamixel_motor/dynamixel_driver/src/dynamixel_driver/')
from dynamixel_io import ChecksumError, DroppedPacketError
from motor_NoROS import Motor




class ReflexOneMotor(Motor):
    def __init__(self, name,ID,_DEFAULT_MOTOR_SPEED,_MAX_MOTOR_SPEED,_MAX_MOTOR_TRAVEL,MOTOR_TO_JOINT_INVERTED,_OVERLOAD_THRESHOLD,MOTOR_TO_JOINT_GEAR_RATIO, MODEL, PROTOCOL):
        super(ReflexOneMotor, self).__init__(name,ID,_DEFAULT_MOTOR_SPEED,_MAX_MOTOR_SPEED,_MAX_MOTOR_TRAVEL,MOTOR_TO_JOINT_INVERTED,_OVERLOAD_THRESHOLD,MOTOR_TO_JOINT_GEAR_RATIO, MODEL, PROTOCOL)
        #print("PRINTING ALL PARAMS:::::::::::::::::",rospy.get_param_names())
        """try:
            self.zero_point = rospy.get_param(self.name + '/zero_point')
        except KeyError:
            rospy.set_param(self.name + '/zero_point', self._motor_msg.raw_angle)
            self.zero_point = rospy.get_param(self.name + '/zero_point')"""
        self.name = name
        self._DEFAULT_MOTOR_SPEED = _DEFAULT_MOTOR_SPEED
        self._MAX_MOTOR_SPEED = _MAX_MOTOR_SPEED
        self._MAX_MOTOR_TRAVEL = _MAX_MOTOR_TRAVEL
        self._OVERLOAD_THRESHOLD = _OVERLOAD_THRESHOLD
        self.MODEL = MODEL
        self.PROTOCOL = PROTOCOL
        self.MOTOR_TO_JOINT_INVERTED = MOTOR_TO_JOINT_INVERTED
        self.ID = ID
        self.MOTOR_TO_JOINT_GEAR_RATIO = MOTOR_TO_JOINT_GEAR_RATIO
        self.zero_point =0
        self.joint_angle = 0
        self.position = 0
        self.velocity = 0
        self.temperature = 0
        self.voltage = 0
        self.goal = 0
        self.moving = 0
        self.load = 0

    def get_motor_feedback(self, dxl_io, print_feedback = False):
        try:
            motor_feedback = dxl_io.get_feedback(self.ID, self.PROTOCOL)
            self.goal = motor_feedback['goal']
            self.position = motor_feedback['position']
            self.error = motor_feedback['error']
            self.speed = motor_feedback['speed']
            self.load = motor_feedback['load']
            self.voltage = motor_feedback['voltage']
            self.temperature = motor_feedback['temperature']
            self.moving = motor_feedback['moving']
            if print_feedback:
                print(motor_feedback)
        except(ChecksumError):
            print 'CheckSumError'
        except(DroppedPacketError):
            print 'DroppedPacketError'



    def set_motor_angle(self, goal_pos, dxl_io):
        '''
        Bounds the given position command and sets it to the motor
        '''
        goal_pos *= self.MOTOR_TO_JOINT_GEAR_RATIO  # Goes from joint radians to motor radians
        if self.MOTOR_TO_JOINT_INVERTED:
            goal_pos *= -1
        goal_pos = self.deg_to_raw(goal_pos) + self.zero_point
        self._set_raw_motor_angle(goal_pos, dxl_io)


    def _check_motor_angle_command(self, angle_command):
        '''
        Returns given command if within the allowable range, returns bounded command if out of range
        '''
        angle_command = self._correct_motor_offset(angle_command)
        if self.MOTOR_TO_JOINT_INVERTED:
            bounded_command = max(min(angle_command, self.zero_point),
                                  self.zero_point - self._MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO)
        else:
            bounded_command = min(max(angle_command, self.zero_point),
                                  self.zero_point + self._MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO)
        return bounded_command
    def _check_motor_point_command(self, point_command):
        if point_command > 1024:
            bounded_command = 1024
            print "Motor ", self.ID, "position command out of bounds"
        elif point_command < 0:
            bounded_command = 0
            print "Command out of bounds"
        else:
            bounded_command = point_command
        return bounded_command

    def set_motor_speed(self, goal_speed, dxl_io):
        '''
        Bounds the given position command and sets it to the motor
        '''
        goal_speed = goal_speed * 19.11 #rad/s to rpm
        goalp_speed = goal_speed / 0.111 # rpm/unit
        val = ((self.ID, int(goal_speed)))
        dxl_io.set_multi_speed([val])

    def reset_motor_speed(self, dxl_io):
        '''
        Resets speed to default
        '''
        goal_speed = self._DEFAULT_MOTOR_SPEED * 19.11 #rad/s to rpm
        goal_speed = goal_speed / 0.111 # rpm/unit
        val = ((self.ID, int(goal_speed)))
        dxl_io.set_multi_speed([val])


    def set_motor_velocity(self, goal_vel, dxl_io):
        '''
        Sets speed and commands finger in or out based on sign of velocity
        '''
        self.set_motor_speed(goal_vel, dxl_io)
        if goal_vel > 0.0:
            max_command = 1024
            goal_pos = max_command-1
            self._set_raw_motor_angle(goal_pos, dxl_io)
        elif goal_vel < 0.0:
            goal_pos = self.zero_point + 1
            self._set_raw_motor_angle(goal_pos, dxl_io)

    def tighten(self,dx_io, tighten_angle=0.05 ):
        '''
        Takes the given angle offset in degrees and tightens the motor
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            tighten_angle *= -1

        self._set_raw_motor_angle(self.deg_to_raw(tighten_angle) + self.position, dx_io)


    def loosen(self, dx_io, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            loosen_angle *= -1
        self._set_raw_motor_angle(self.position - self.deg_to_raw(loosen_angle), dx_io)



    def _receive_state_cb(self, dxl_io):
        # Calculate joint angle from motor angle
        if self.MOTOR_TO_JOINT_INVERTED:
            joint_angle = self.zero_point - self.position
        else:
            joint_angle =  self.position - self.zero_point
        self.joint_angle = joint_angle / self.MOTOR_TO_JOINT_GEAR_RATIO

        self._handle_motor_load(self.load, dxl_io)

        return {'joint_angle': self.joint_angle, 'raw_angle': self.position,
        'velocity': self.velocity, 'load': self.load, 'temperature': self.temperature}

    def _handle_motor_load(self, load, dxl_io):
        load_filter = 0.25  # Rolling filter of noisy data
        if not self.MOTOR_TO_JOINT_INVERTED:
            load *= -1
            self.load = load_filter * load + (1 - load_filter) * self.load
        if self._in_control_force_mode:
            pass
            self._control_force(self.load, dxl_io, k=3.0*0.025)
            self._loosen_if_overloaded(self.load)

    def _set_local_motor_zero_point(self):
        pass
        self.zero_point = self.position
#rospy.set_param(self.name + '/zero_point', self._motor_msg.raw_angle)

    def _set_raw_motor_angle(self, goal_pos, dx_io):
        """
        Changing angle units to 100 degrees so 3.00 would be 300 degrees
        """
        goal_point =int(self._check_motor_point_command(goal_pos))
        dx_io.set_position(self.ID, goal_point)



    def _correct_motor_offset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command
    def deg_to_raw(self,goal_pos):
        goal_angle = goal_pos*57.3 #deg per rad
        goal_point = goal_angle / 0.29 # degrees/encoder_tick on AX-12 and XL-320

        return goal_point

    def enable_torque(self, dxl_io):
        dxl_io.set_torque_enabled(self.ID, 1)

    def disable_torque(self):
        dxl_io.set_torque_enabled(self.ID, 0)
