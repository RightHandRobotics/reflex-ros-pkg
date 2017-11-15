#!/usr/bin/env python

# Example code for a script using the ReFlex Takktile hand
# Note: you must connect a hand by running "roslaunch reflex reflex.launch" before you can run this script


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


def main():
    ##################################################################################################################
    rospy.init_node('ExampleHandNode')

    # Services can automatically call hand calibration
    calibrate_fingers = rospy.ServiceProxy('/reflex_takktile/calibrate_fingers', Empty)
    calibrate_tactile = rospy.ServiceProxy('/reflex_takktile/calibrate_tactile', Empty)
    
    # Services can set tactile thresholds and enable tactile stops
    enable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/enable_tactile_stops', Empty)
    disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/disable_tactile_stops', Empty)
    set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile/set_tactile_threshold', SetTactileThreshold)

    # This collection of publishers can be used to command the hand
    command_pub = rospy.Publisher('/reflex_takktile/command', Command, queue_size=1)
    pos_pub = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
    vel_pub = rospy.Publisher('/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)
    force_pub = rospy.Publisher('/reflex_takktile/command_motor_force', ForceCommand, queue_size=1)

    # Constantly capture the current hand state
    rospy.Subscriber('/reflex_takktile/hand_state', Hand, hand_state_cb)

    ##################################################################################################################
    # Calibrate the fingers (make sure hand is opened in zero position)
    raw_input("== When ready to calibrate the hand, press [Enter]\n")
    calibrate_fingers()
## Can call... rostopic echo reflex_takktile/hand_state/finger


 ##################################################################################################################
    raw_input("Press Enter to start test...\n")    
    speedFactor = 0.9
    rospy.sleep(2.3*speedFactor)
    pos_pub.publish(PoseCommand()) # preshape in
    pos_pub.publish(PoseCommand(f1=2.75, f2=2.75, f3=2.75, preshape=0.0)) #close gripper
    rospy.sleep(4.7*speedFactor)
    pos_pub.publish(PoseCommand())  #open gripper
    #rospy.sleep(2.5*speedFactor) # Add 2.5s since it opens much faster
    rospy.sleep(4.3*speedFactor) # wait longer after openning gripper +1.5s
    pos_pub.publish(PoseCommand(f1=2.75, f2=2.75, f3=2.75, preshape=0.0)) #close gripper
    rospy.sleep(4.3*speedFactor)
    pos_pub.publish(PoseCommand())  #open gripper
    rospy.sleep(2.5*speedFactor)  # wait longer after openning gripper +1.5s
    pos_pub.publish(PoseCommand(preshape=1.57)) #preshape out
    pos_pub.publish(PoseCommand(f1=1.6, f2=1.6, f3=1.7, preshape=1.6)) #partially close gripper
    rospy.sleep(4.2*speedFactor)
    pos_pub.publish(PoseCommand(f1=2.75, f2=2.75, f3=1.7, preshape=1.6))
    rospy.sleep(5.5*speedFactor)
    pos_pub.publish(PoseCommand())  #open gripper
    rospy.sleep(6.1*speedFactor);

    """
    raw_input("Press Enter to start test...\n")    
    for i in range(10):
        pos_pub.publish(PoseCommand(f1=2.75, f2=2.75, f3=2.75, preshape=0.0))
        rospy.sleep(1)
	pos_pub.publish(PoseCommand())
	rospy.sleep(1)
	

    raw_input("...\n")

    ##################################################################################################################
    # Demonstration of preshape joint
    raw_input("== When ready to test preshape joint, hit [Enter]\n")
    for i in range (10):
    	pos_pub.publish(PoseCommand(preshape=1.57))
	rospy.sleep(1.0)
	pos_pub.publish(PoseCommand(f1=2.0, f2=2.0, f3=0.0, preshape=1.6))
    	rospy.sleep(2.0)
    	pos_pub.publish(PoseCommand())
    	rospy.sleep(2.0)
    """

    raw_input("Done...\n")
    

def hand_state_cb(data):
    global hand_state
    hand_state = data


if __name__ == '__main__':
    main()
