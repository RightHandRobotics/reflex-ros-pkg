#!/usr/bin/env python

# Example code for a script using the ReFlex Takktile hand
# Note: you must connect a hand by running "roslaunch reflex reflex.launch" before you can run this script


from math import pi, cos

import rospy
from std_srvs.srv import Empty

from reflex_msgs2.msg import Command
from reflex_msgs2.msg import PoseCommand
from reflex_msgs2.msg import VelocityCommand
from reflex_msgs2.msg import ForceCommand
from reflex_msgs2.msg import Hand
from reflex_msgs2.msg import FingerPressure
from reflex_msgs2.srv import SetTactileThreshold, SetTactileThresholdRequest


hand_state = Hand()


def main():
    ##################################################################################################################
    rospy.init_node('ExampleHandNode')

    # Services can automatically call hand calibration
    calibrate_fingers = rospy.ServiceProxy('/reflex_takktile2/calibrate_fingers', Empty)
    calibrate_tactile = rospy.ServiceProxy('/reflex_takktile2/calibrate_tactile', Empty)
    
    # Services can set tactile thresholds and enable tactile stops
    enable_tactile_stops = rospy.ServiceProxy('/reflex_takktile2/enable_tactile_stops', Empty)
    disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile2/disable_tactile_stops', Empty)
    set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile2/set_tactile_threshold', SetTactileThreshold)

    # This collection of publishers can be used to command the hand
    command_pub = rospy.Publisher('/reflex_takktile2/command', Command, queue_size=1)
    pos_pub = rospy.Publisher('/reflex_takktile2/command_position', PoseCommand, queue_size=1)
    vel_pub = rospy.Publisher('/reflex_takktile2/command_velocity', VelocityCommand, queue_size=1)
    force_pub = rospy.Publisher('/reflex_takktile2/command_motor_force', ForceCommand, queue_size=1)

    # Constantly capture the current hand state
    rospy.Subscriber('/reflex_takktile2/hand_state', Hand, hand_state_cb)

    ##################################################################################################################
    # Calibrate the fingers (make sure hand is opened in zero position)
    raw_input("== When ready to calibrate the hand, press [Enter]\n")
    calibrate_fingers()
    raw_input("...\n")


    while True:
        demo_type = input("1: Wiggle, 2: Preshape, 3: Guarded, 4:Velocity, 5:Force, 6: Enable TakkTile, 7: Disable TakkTile, 8: Calibrate Fingers 9:Exit\n")

        if demo_type == 1:
            # Demonstration of position control
            wiggle_cycles = input ("How many times would you like to wiggle?\n")
            raw_input("== When ready to wiggle fingers with position control, hit [Enter]\n")
            for i in range(wiggle_cycles):
                pos_pub.publish(PoseCommand(f1=3.5, f2=3.5, f3=3.5, preshape=0.0))
                rospy.sleep(1.0)
                pos_pub.publish(PoseCommand())
                rospy.sleep(1.0)
                
        if demo_type == 2:
            # Demonstration of preshape joint
            preshape_cycles = input ("How many times would you like to test the preshape?\n")
            raw_input("== When ready to test preshape joint, hit [Enter]\n")
            for i in range (preshape_cycles):
                pos_pub.publish(PoseCommand(preshape=1.57))
                rospy.sleep(1.0)
                pos_pub.publish(PoseCommand(f1=3.0, f2=3.0, f3=0, preshape=1.57))
                rospy.sleep(2.0)
                pos_pub.publish(PoseCommand())
                rospy.sleep(2.0)

    
        if demo_type == 3:
             # Demonstration of tactile feedback and setting sensor thresholds
            raw_input("== When ready to calibrate tactile sensors and close until contact, hit [Enter]\n")
            calibrate_tactile()
            enable_tactile_stops()
            f1 = FingerPressure([10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 1000])
            f2 = FingerPressure([15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 1000])
            f3 = FingerPressure([20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 1000])
            threshold = SetTactileThresholdRequest([f1, f2, f3])
            set_tactile_threshold(threshold)
            vel_pub.publish(VelocityCommand(f1=.75, f2=.75, f3=.75, preshape=0.0))
            rospy.sleep(3.0)
            pos_pub.publish(PoseCommand())
            disable_tactile_stops()

        if demo_type == 4:
            raw_input("== When ready to test velocity control, hit [Enter]\n")
            vel_pub.publish(VelocityCommand(f1=1.0, f2=2.0, f3=3.0, preshape=0.0))
            rospy.sleep(5.0)
            pos_pub.publish(PoseCommand())
            rospy.sleep(1.0)

        if demo_type == 5:
            raw_input("== When ready to test Force control, hit [Enter]\n")
            force_pub.publish(ForceCommand(f1=200, f2=200, f3=200, preshape=0.0))
            rospy.sleep(5.0)
            pos_pub.publish(PoseCommand())
            rospy.sleep(1.0)            

        if demo_type == 6:
            enable_tactile_stops()
            print "TakkTile Sensors Enabled"

        if demo_type == 7:
            disable_tactile_stops()
            print "TakkTile Sensors Disabled"

            
        if demo_type == 8:
            calibrate_fingers()
            calibrate_tactile()
            print "Finger Calibration Complete"
       
        if demo_type == 9:
            break

    
def hand_state_cb(data):
    global hand_state
    hand_state = data


if __name__ == '__main__':
    main()
