#!/usr/bin/env python

# Test code for the ReFlex hand
# Note: you must connect a hand by running "roslaunch reflex
# reflex.launch" before you can run this script

import rospy
from std_srvs.srv import Empty

# Imports the code to control the hand using smart actions
from reflex import reflex_smarts
rospy.init_node('TestHandNode')

# Starts up hand and publishes the command data
reflex_hand = reflex_smarts.ReFlex_Smarts()
rospy.sleep(1)
rospy.wait_for_service('/zero_tactile')
zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)
rospy.wait_for_service('/zero_fingers')
zero_fingers = rospy.ServiceProxy('/zero_fingers', Empty)

###############################################################################
# Calibrate the hand and check the raw values
raw_input("When ready to calibrate the hand, press [Enter] ")
zero_fingers()
raw_input("Open a terminal and rostopic echo the /reflex/debug_info\n\
topic. Check that the raw angle values are correct:\n\
-- F1: less than 900 -- F2: greater than 3200 -- F3: less than 900 --\n\
If all is good, hit [Enter]. If not, cancel the script and redo tendons")

###############################################################################
# Check the freedom of movement of the fingers
raw_input("When ready to check finger freedom of movement, press [Enter] ")
for finger in range(3):
    reflex_hand.move_finger(finger, 1.5)
    rospy.sleep(4)
    reflex_hand.open(finger)
    rospy.sleep(4)
reflex_hand.command_smarts(1, 'close')
rospy.sleep(4)
reflex_hand.command_smarts(1, 'open')
raw_input("If all went well, continue to next test by pressing [Enter] ")

###############################################################################
# Check the spherical freedom of motion
raw_input("When ready to check spherical freedom of movement, press [Enter] ")
reflex_hand.set_spherical()
rospy.sleep(4)
for finger in range(3):
    reflex_hand.move_finger(finger, 1.5)
    rospy.sleep(4)
reflex_hand.command_smarts(1, 'open')
raw_input("If all went well, continue to next test by pressing [Enter] ")

###############################################################################
# Check the pinch freedom of motion
raw_input("When ready to check pinch freedom of movement, press [Enter] ")
reflex_hand.set_pinch()
rospy.sleep(4)
for finger in range(2):
    reflex_hand.close(finger)
    rospy.sleep(4)
    reflex_hand.open(finger)
    rospy.sleep(4)
for finger in range(2):
    reflex_hand.move_finger(finger, 1.5)
rospy.sleep(4)
reflex_hand.command_smarts(1, 'open')
rospy.sleep(4)
reflex_hand.set_cylindrical()
raw_input("If all went well, continue to next test by pressing [Enter] ")

###############################################################################

