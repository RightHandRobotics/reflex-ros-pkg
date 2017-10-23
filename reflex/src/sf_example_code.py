#!/usr/bin/env python

# Example code for a script using the ReFlex Takktile hand
# Note: you must connect a hand by running "roslaunch reflex reflex.launch" before you can run this script


from math import pi, cos

import rospy
from std_srvs.srv import Empty

from reflex_msgs.msg import Command
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
# from reflex_msgs.msg import ForceCommand
from reflex_msgs.msg import Hand


hand_state = Hand()


def main():
    ##################################################################################################################
    rospy.init_node('ExampleHandNode')

    # Services can automatically call hand calibration
    calibrate_fingers = rospy.ServiceProxy('/reflex_sf/calibrate_fingers', Empty)

    # This collection of publishers can be used to command the hand
    command_pub = rospy.Publisher('/reflex_sf/command', Command, queue_size=1)
    pos_pub = rospy.Publisher('/reflex_sf/command_position', PoseCommand, queue_size=1)
    vel_pub = rospy.Publisher('/reflex_sf/command_velocity', VelocityCommand, queue_size=1)
    # force_pub = rospy.Publisher('/reflex_sf/command_motor_force', ForceCommand, queue_size=1)

    # Constantly capture the current hand state
    rospy.Subscriber('/reflex_sf/hand_state', Hand, hand_state_cb)

    ##################################################################################################################
    # Calibrate the fingers (make sure hand is opened in zero position)
    raw_input("== When ready to calibrate the hand, press [Enter]\n")
    print('Go to the window where reflex_sf was run, you will see the calibration prompts')
    calibrate_fingers()
    raw_input("...\n")

    ##################################################################################################################
    # Demonstration of position control
    raw_input("== When ready to wiggle fingers with position control, hit [Enter]\n")
    for i in range(60):
        setpoint = (-cos(i / 5.0) + 1) * 1.75
        pos_pub.publish(PoseCommand(f1=setpoint, f2=setpoint, f3=setpoint, preshape=0.0))
        rospy.sleep(0.25)
    raw_input("...\n")

    ##################################################################################################################
    # Demonstration of preshape joint
    raw_input("== When ready to test preshape joint, hit [Enter]\n")
    pos_pub.publish(PoseCommand(preshape=1.57))
    rospy.sleep(2.0)
    pos_pub.publish(PoseCommand())
    rospy.sleep(2.0)
    raw_input("...\n")

    ##################################################################################################################
    # Demonstration of velocity control - variable closing speed
    raw_input("== When ready to open and close fingers with velocity control, hit [Enter]\n")
    for i in range(3):
        pos_pub.publish(PoseCommand(f1=i, f2=i, f3=i, preshape=0.0))
        rospy.sleep(2.0)
        setpoint = 5.0 - (i * 2.25)
        vel_pub.publish(VelocityCommand(f1=setpoint, f2=setpoint, f3=setpoint, preshape=0.0))
        rospy.sleep(7.0 - setpoint)
    raw_input("...\n")
    pos_pub.publish(PoseCommand())

    ##################################################################################################################
    # Demonstration of blended control - asymptotic approach to goal - uses hand_state
    raw_input("== When ready to approach target positions with blended control, hit [Enter]\n")
    pose = PoseCommand(f1=3.5, f2=2.25, f3=1.0, preshape=0.0)
    velocity = VelocityCommand()
    for i in range(1, 5):
        velocity.f1 = round(pose.f1 - hand_state.motor[0].joint_angle, 1) + 0.5
        velocity.f2 = round(pose.f2 - hand_state.motor[1].joint_angle, 1) + 0.5
        velocity.f3 = round(pose.f3 - hand_state.motor[2].joint_angle, 1) + 0.5
        command_pub.publish(Command(pose, velocity))
        rospy.sleep(0.75)
    raw_input("...\n")

    ##################################################################################################################
    # # Demonstration of force control - square wave
    # # NEEDS TESTING
    # raw_input("== When ready to feel variable force control, hit [Enter]\n")
    # raw_input("== Putting your arm or hand in the hand will allow you to feel the effect [Enter]\n")
    # pos_pub.publish(PoseCommand(f1=1.5, f2=1.5, f3=1.5, preshape=0.0))
    # rospy.sleep(2.0)
    # force_pub.publish(ForceCommand(f1=300.0))
    # rospy.sleep(5.0)
    # force_pub.publish(ForceCommand(f1=100.0))
    # rospy.sleep(3.0)
    # force_pub.publish(ForceCommand(f2=300.0))
    # rospy.sleep(5.0)
    # force_pub.publish(ForceCommand(f2=100.0))
    # rospy.sleep(3.0)
    # force_pub.publish(ForceCommand(f3=300.0))
    # rospy.sleep(5.0)
    # force_pub.publish(ForceCommand(f3=100.0))
    # rospy.sleep(3.0)
    # vel_pub.publish(VelocityCommand(f1=-5.0, f2=-5.0, f3=-5.0, preshape=0.0))
    # rospy.sleep(2.0)
    # raw_input("...\n")


def hand_state_cb(data):
    global hand_state
    hand_state = data


if __name__ == '__main__':
    main()
