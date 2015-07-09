#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with a ReFlex SF hand


from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospy
import rospkg
from std_msgs.msg import Float64

from reflex_sf_msgs.msg import SFCommand
from reflex_sf_msgs.msg import SFPose
from reflex_sf_msgs.msg import SFVelocity
from motor import Motor


class ReflexSFHand(object):
    def __init__(self):
        rospy.init_node('reflex_sf')
        rospy.loginfo('Starting up the ReFlex SF hand')
        self.motors = {'/reflex_sf_f1': Motor('/reflex_sf_f1'),
                       '/reflex_sf_f2': Motor('/reflex_sf_f2'),
                       '/reflex_sf_f3': Motor('/reflex_sf_f3'),
                       '/reflex_sf_preshape': Motor('/reflex_sf_preshape')}
        rospy.Subscriber('/reflex_sf/command', SFCommand, self.receiveCmdCb)
        rospy.Subscriber('/reflex_sf/command_position', SFPose, self.receivePosCmdCb)
        rospy.Subscriber('/reflex_sf/command_velocity', SFVelocity, self.receiveVelCmdCb)
        rospy.loginfo('ReFlex SF hand has started, waiting for commands...')

    def receiveCmdCb(self, data):
        self.setSpeed(data.velocity)
        self.setPosition(data.pose)

    def setPosition(self, data):
        self.motors['/reflex_sf_f1'].setMotorPosition(data.f1)
        self.motors['/reflex_sf_f2'].setMotorPosition(data.f2)
        self.motors['/reflex_sf_f3'].setMotorPosition(data.f3)
        self.motors['/reflex_sf_preshape'].setMotorPosition(data.preshape)

    def setSpeed(self, data):
        self.motors['/reflex_sf_f1'].setSpeed(data.f1)
        self.motors['/reflex_sf_f2'].setSpeed(data.f2)
        self.motors['/reflex_sf_f3'].setSpeed(data.f3)
        self.motors['/reflex_sf_preshape'].setSpeed(data.preshape)

    def disableTorque(self):
        for motor in self.motors:
            self.motors[motor].disableTorque()

    def enableTorque(self):
        for motor in self.motors:
            self.motors[motor].enableTorque()


def main():
    hand = ReflexSFHand()
    rospy.on_shutdown(hand.disableTorque)
    rospy.spin()


if __name__ == '__main__':
    main()
