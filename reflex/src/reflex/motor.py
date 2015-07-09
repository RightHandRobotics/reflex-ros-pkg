#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with components of the ReFlex SF hand
#

from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetSpeed
import rospy
from std_msgs.msg import Float64


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_sf_f1
        '''
        self.name = name[1:]
        self.zero_point = rospy.get_param(self.name + '/zero_point')
        self.ANGLE_RANGE = rospy.get_param(self.name + '/angle_range')
        self.JOINT_SPEED = rospy.get_param(self.name + '/joint_speed')
        self.MAX_SPEED = rospy.get_param(self.name + '/max_speed')
        self.FLIPPED = rospy.get_param(self.name + '/flipped')
        self.current_raw_position = 0.0
        self.current_pos = 0.0
        self.load = 0
        self.OVERLOAD_THRESHOLD = 0.2  # overload threshold to avoid thermal issues
        self.TAU = 0.1  # time constant of lowpass filter
        self.pub = rospy.Publisher(name + '/command', Float64, queue_size=10)
        self.set_speed_service = rospy.ServiceProxy(name + '/set_speed', SetSpeed)
        self.set_speed_service(self.JOINT_SPEED)
        self.torque_enable_service = rospy.ServiceProxy(name + '/torque_enable', TorqueEnable)
        self.torque_enabled = True
        self.sub = rospy.Subscriber(name + '/state', JointState, self.receiveStateCb)

    def setMotorZeroPoint(self):
        self.zero_point = self.current_raw_position
        rospy.set_param(self.name + '/zero_point', self.current_raw_position)

    def getRawCurrentPosition(self):
        return self.current_raw_position

    def getCurrentPosition(self):
        return self.current_position

    def setRawMotorPosition(self, goal_pos):
        self.pub.publish(goal_pos)

    def setMotorPosition(self, goal_pos):
        '''
        Bounds the given motor command and sets it to the motor
        '''
        self.set_speed_service(self.JOINT_SPEED)
        self.pub.publish(self.checkMotorPositionCommand(goal_pos))

    def checkMotorPositionCommand(self, angle_command):
        '''
        Returns given command if within the allowable range, returns bounded command if out of range
        '''
        angle_command = self.correctMotorOffset(angle_command)
        if self.FLIPPED:
            bounded_command = max(min(angle_command, self.zero_point), self.zero_point - self.ANGLE_RANGE)
        else:
            bounded_command = min(max(angle_command, self.zero_point), self.zero_point + self.ANGLE_RANGE)
        return bounded_command

    def setMotorVelocity(self, goal_vel):
        '''
        Bounds the given motor command and sets it to the motor. Commands finger in or out based on sign of velocity
        '''
        self.set_speed_service(self.checkMotorVelocityCommand(goal_vel))
        if goal_vel > 0.0:
            self.pub.publish(self.checkMotorPositionCommand(self.ANGLE_RANGE))
        elif goal_vel < 0.0:
            self.pub.publish(self.checkMotorPositionCommand(0.0))

    def checkMotorVelocityCommand(self, vel_command):
        '''
        Returns given command if within the allowable range, returns bounded command if out of range
        '''
        bounded_command = min(max(abs(vel_command), 0), self.MAX_SPEED)
        return bounded_command

    def correctMotorOffset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.FLIPPED:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command

    def enableTorque(self):
        self.torque_enabled = True
        self.torque_enable_service(True)

    def disableTorque(self):
        self.torque_enabled = False
        self.torque_enable_service(False)

    def loosenIfOverloaded(self, load, velocity):
        if abs(load) > self.OVERLOAD_THRESHOLD and True:
            print("Motor %s overloaded at %f, loosening" % (self.name, load))
            self.loosen()

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        if self.FLIPPED:
            tighten_angle *= -1
        self.setRawMotorPosition(self.current_raw_position + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.FLIPPED:
            loosen_angle *= -1
        self.setRawMotorPosition(self.current_raw_position - loosen_angle)

    def receiveStateCb(self, data):
        self.current_raw_position = data.current_pos
        self.load = self.TAU * data.load + (1 - self.TAU) * self.load  # Rolling filter of noisy data
        if self.FLIPPED:
            self.current_position = self.zero_point - self.current_raw_position
        else:
            self.current_position = self.current_raw_position - self.zero_point
        self.loosenIfOverloaded(self.load, data.velocity)
