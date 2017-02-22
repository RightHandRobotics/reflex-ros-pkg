#!/usr/bin/env python

# Example code for a script using the ReFlex Takktile hand
# Note: you must connect a hand by running "roslaunch reflex reflex_takktile.launch" before you can run this script


from math import pi, cos

import rospy
from std_srvs.srv import Empty

from reflex_msgs.msg import Command
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import ForceCommand
from reflex_msgs.msg import Hand
from reflex_msgs.msg import FingerPressure
from reflex_msgs.srv import SetTactileThreshold, SetTactileThresholdRequest


hand_state = Hand()

# TODO: update script for two hands (probably need a second hand object)

def main():
    ##################################################################################################################
    rospy.init_node('ExampleHandNode')

    # Hand 1

    print("Initiallizing Hand 1\n")

    # Services can automatically call hand calibration
    calibrate_fingers_hand1 = rospy.ServiceProxy('/reflex_takktile/calibrate_fingers', Empty)
    calibrate_tactile_hand1 = rospy.ServiceProxy('/reflex_takktile/calibrate_tactile', Empty)
    
    # Services can set tactile thresholds and enable tactile stops
    enable_tactile_stops_hand1 = rospy.ServiceProxy('/reflex_takktile/enable_tactile_stops', Empty)
    disable_tactile_stops_hand1 = rospy.ServiceProxy('/reflex_takktile/disable_tactile_stops', Empty)
    set_tactile_threshold_hand1 = rospy.ServiceProxy('/reflex_takktile/set_tactile_threshold', SetTactileThreshold)

    # This collection of publishers can be used to command the hand
    command_pub_hand1 = rospy.Publisher('/reflex_takktile/command', Command, queue_size=1)
    pos_pub_hand1 = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
    vel_pub_hand1 = rospy.Publisher('/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)
    force_pub_hand1 = rospy.Publisher('/reflex_takktile/command_motor_force', ForceCommand, queue_size=1)

    # Constantly capture the current hand state
    rospy.Subscriber('/reflex_takktile/hand_state', Hand, hand_state_cb)

    ##################################################################################################################

    # Hand 2

    print("Initiallizing Hand 2\n")

    # Services can automatically call hand calibration
    calibrate_fingers_hand2 = rospy.ServiceProxy('/hand2/reflex_takktile/calibrate_fingers', Empty)
    calibrate_tactile_hand2 = rospy.ServiceProxy('/hand2/reflex_takktile/calibrate_tactile', Empty)
    
    # Services can set tactile thresholds and enable tactile stops
    enable_tactile_stops_hand2 = rospy.ServiceProxy('/hand2/reflex_takktile/enable_tactile_stops', Empty)
    disable_tactile_stops_hand2 = rospy.ServiceProxy('/hand2/reflex_takktile/disable_tactile_stops', Empty)
    set_tactile_threshold_hand2 = rospy.ServiceProxy('/hand2/reflex_takktile/set_tactile_threshold', SetTactileThreshold)

    # This collection of publishers can be used to command the hand
    command_pub_hand2 = rospy.Publisher('/hand2/reflex_takktile/command', Command, queue_size=1)
    pos_pub_hand2 = rospy.Publisher('/hand2/reflex_takktile/command_position', PoseCommand, queue_size=1)
    vel_pub_hand2 = rospy.Publisher('/hand2/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)
    force_pub_hand2 = rospy.Publisher('/hand2/reflex_takktile/command_motor_force', ForceCommand, queue_size=1)

    # Constantly capture the current hand state
    rospy.Subscriber('/hand2/reflex_takktile/hand_state', Hand, hand_state_cb)

    ##################################################################################################################
    # Calibrate the fingers (make sure hand is opened in zero position)
    raw_input("== When ready to calibrate Hand 1, press [Enter]\n")
    calibrate_fingers_hand1()
    raw_input("... [Enter]\n")

    raw_input("== When ready to calibrate Hand 2, press [Enter]\n")
    calibrate_fingers_hand2()
    raw_input("... [Enter]\n")

    ##################################################################################################################
    # Demonstration of position control
    raw_input("== When ready to wiggle fingers on Hand 1 with position control, hit [Enter]\n")
    for i in range(100):
        setpoint = (-cos(i / 15.0) + 1) * 1.75
        pos_pub_hand1.publish(PoseCommand(f1=setpoint, f2=setpoint, f3=setpoint, preshape=0.0))
        rospy.sleep(0.1)
    raw_input("... [Enter]\n")
    pos_pub_hand1.publish(PoseCommand())

    raw_input("== When ready to wiggle fingers on Hand 2 with position control, hit [Enter]\n")
    for i in range(100):
        setpoint = (-cos(i / 15.0) + 1) * 1.75
        pos_pub_hand2.publish(PoseCommand(f1=setpoint, f2=setpoint, f3=setpoint, preshape=0.0))
        rospy.sleep(0.1)
    raw_input("... [Enter]\n")
    pos_pub_hand2.publish(PoseCommand())

    ##################################################################################################################
    # Demonstration of preshape joint
    raw_input("== When ready to test preshape joint of Hand 1, hit [Enter]\n")
    pos_pub_hand1.publish(PoseCommand(preshape=1.57))
    rospy.sleep(2.0)
    pos_pub_hand1.publish(PoseCommand())
    rospy.sleep(2.0)
    raw_input("... [Enter]\n")

    raw_input("== When ready to test preshape joint of Hand 2, hit [Enter]\n")
    pos_pub_hand2.publish(PoseCommand(preshape=1.57))
    rospy.sleep(2.0)
    pos_pub_hand2.publish(PoseCommand())
    rospy.sleep(2.0)
    raw_input("... [Enter]\n")

    ##################################################################################################################
    # Demonstration of velocity control - variable closing speed
    raw_input("== When ready to open and close fingers of Hand 1 with velocity control, hit [Enter]\n")
    vel_pub_hand1.publish(VelocityCommand(f1=3.0, f2=2.0, f3=1.0, preshape=0.0))
    rospy.sleep(4.0)
    vel_pub_hand1.publish(VelocityCommand(f1=-1.0, f2=-2.0, f3=-3.0, preshape=0.0))
    rospy.sleep(4.0)
    raw_input("... [Enter]\n")
    pos_pub_hand1.publish(PoseCommand())

    raw_input("== When ready to open and close fingers of Hand 2 with velocity control, hit [Enter]\n")
    vel_pub_hand2.publish(VelocityCommand(f1=3.0, f2=2.0, f3=1.0, preshape=0.0))
    rospy.sleep(4.0)
    vel_pub_hand2.publish(VelocityCommand(f1=-1.0, f2=-2.0, f3=-3.0, preshape=0.0))
    rospy.sleep(4.0)
    raw_input("... [Enter]\n")
    pos_pub_hand2.publish(PoseCommand())

    ##################################################################################################################
    # Demonstration of blended control - asymptotic approach to goal - uses hand_state
    raw_input("== When ready to approach target positions with blended control using Hand 1, hit [Enter]\n")
    pose = PoseCommand(f1=3.5, f2=2.25, f3=1.0, preshape=0.0)
    velocity = VelocityCommand()
    for i in range(1, 5):
        velocity.f1 = round(pose.f1 - hand_state.motor[0].joint_angle, 1) + 0.25
        velocity.f2 = round(pose.f2 - hand_state.motor[1].joint_angle, 1) + 0.25
        velocity.f3 = round(pose.f3 - hand_state.motor[2].joint_angle, 1) + 0.25
        command_pub_hand1.publish(Command(pose, velocity))
        rospy.sleep(0.4)
    raw_input("... [Enter]\n")
    pos_pub_hand1.publish(PoseCommand())

    raw_input("== When ready to approach target positions with blended control using Hand 2, hit [Enter]\n")
    pose = PoseCommand(f1=3.5, f2=2.25, f3=1.0, preshape=0.0)
    velocity = VelocityCommand()
    for i in range(1, 5):
        velocity.f1 = round(pose.f1 - hand_state.motor[0].joint_angle, 1) + 0.25
        velocity.f2 = round(pose.f2 - hand_state.motor[1].joint_angle, 1) + 0.25
        velocity.f3 = round(pose.f3 - hand_state.motor[2].joint_angle, 1) + 0.25
        command_pub_hand2.publish(Command(pose, velocity))
        rospy.sleep(0.4)
    raw_input("... [Enter]\n")
    pos_pub_hand2.publish(PoseCommand())

    ##################################################################################################################
    # # Demonstration of force control - square wave
    raw_input("== When ready to feel variable force control using Hand 1, hit [Enter]\n")
    raw_input("== Putting your arm or hand in the hand will allow you to feel the effect [Enter]\n")
    pos_pub_hand1.publish(PoseCommand(f1=1.5, f2=1.5, f3=1.5, preshape=0.0))
    rospy.sleep(2.0)
    print("Tightening finger 1")
    force_pub_hand1.publish(ForceCommand(f1=300.0))
    rospy.sleep(5.0)
    print("Partially loosen finger 1")
    force_pub_hand1.publish(ForceCommand(f1=100.0))
    rospy.sleep(1.0)
    print("Tightening finger 2")
    force_pub_hand1.publish(ForceCommand(f2=300.0))
    rospy.sleep(5.0)
    print("Partially loosen finger 2")
    force_pub_hand1.publish(ForceCommand(f2=100.0))
    rospy.sleep(1.0)
    print("Tightening finger 3")
    force_pub_hand1.publish(ForceCommand(f3=300.0))
    rospy.sleep(5.0)
    print("Partially loosen finger 3")
    force_pub_hand1.publish(ForceCommand(f3=100.0))
    rospy.sleep(1.0)
    vel_pub_hand1.publish(VelocityCommand(f1=-5.0, f2=-5.0, f3=-5.0, preshape=0.0))
    rospy.sleep(2.0)
    raw_input("... [Enter]\n")

    raw_input("== When ready to feel variable force control using Hand 2, hit [Enter]\n")
    raw_input("== Putting your arm or hand in the hand will allow you to feel the effect [Enter]\n")
    pos_pub_hand2.publish(PoseCommand(f1=1.5, f2=1.5, f3=1.5, preshape=0.0))
    rospy.sleep(2.0)
    print("Tightening finger 1")
    force_pub_hand2.publish(ForceCommand(f1=300.0))
    rospy.sleep(5.0)
    print("Partially loosen finger 1")
    force_pub_hand2.publish(ForceCommand(f1=100.0))
    rospy.sleep(1.0)
    print("Tightening finger 2")
    force_pub_hand2.publish(ForceCommand(f2=300.0))
    rospy.sleep(5.0)
    print("Partially loosen finger 2")
    force_pub_hand2.publish(ForceCommand(f2=100.0))
    rospy.sleep(1.0)
    print("Tightening finger 3")
    force_pub_hand2.publish(ForceCommand(f3=300.0))
    rospy.sleep(5.0)
    print("Partially loosen finger 3")
    force_pub_hand2.publish(ForceCommand(f3=100.0))
    rospy.sleep(1.0)
    vel_pub_hand2.publish(VelocityCommand(f1=-5.0, f2=-5.0, f3=-5.0, preshape=0.0))
    rospy.sleep(2.0)
    raw_input("... [Enter]\n")

    ##################################################################################################################
    # Demonstration of tactile feedback and setting sensor thresholds
    raw_input("== When ready to calibrate tactile sensors of Hand 1 and close until contact, hit [Enter]\n")
    calibrate_tactile_hand1()
    enable_tactile_stops_hand1()
    f1 = FingerPressure([10, 10, 10, 10, 10, 10, 10, 10, 1000])
    f2 = FingerPressure([15, 15, 15, 15, 15, 15, 15, 15, 1000])
    f3 = FingerPressure([20, 20, 20, 20, 20, 20, 20, 20, 1000])
    threshold = SetTactileThresholdRequest([f1, f2, f3])
    set_tactile_threshold_hand1(threshold)
    vel_pub_hand1.publish(VelocityCommand(f1=1.0, f2=1.0, f3=1.0, preshape=0.0))
    rospy.sleep(3.0)
    raw_input("... [Enter]\n")
    pos_pub_hand1.publish(PoseCommand())
    disable_tactile_stops_hand1()

    raw_input("== When ready to calibrate tactile sensors of Hand 2 and close until contact, hit [Enter]\n")
    calibrate_tactile_hand2()
    enable_tactile_stops_hand2()
    f1 = FingerPressure([10, 10, 10, 10, 10, 10, 10, 10, 1000])
    f2 = FingerPressure([15, 15, 15, 15, 15, 15, 15, 15, 1000])
    f3 = FingerPressure([20, 20, 20, 20, 20, 20, 20, 20, 1000])
    threshold = SetTactileThresholdRequest([f1, f2, f3])
    set_tactile_threshold_hand2(threshold)
    vel_pub_hand2.publish(VelocityCommand(f1=1.0, f2=1.0, f3=1.0, preshape=0.0))
    rospy.sleep(3.0)
    raw_input("... [Enter]\n")
    pos_pub_hand2.publish(PoseCommand())
    disable_tactile_stops_hand2()



def hand_state_cb(data):
    global hand_state
    hand_state = data


if __name__ == '__main__':
    main()
