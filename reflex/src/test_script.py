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
std_wait = 2

###############################################################################
# Calibrate the hand and check the raw values
raw_input("== When ready to calibrate the hand, press [Enter]\n")
zero_fingers()
raw_input("== Open a terminal and rostopic echo the /reflex/motor_debug\n\
== topic. Check that the raw angle values are correct:\n\
== F1: less than 900 -- F2: greater than 3200 -- F3: less than 900\n\
== If all is good, hit [Enter]. If not, cancel the script and redo tendons\n")

###############################################################################
# Check the freedom of movement of the fingers
raw_input("== When ready to check finger freedom of movement, press [Enter]\n")
for finger in range(3):
    rospy.sleep(std_wait)
    reflex_hand.move_finger(finger, 2)
    rospy.sleep(std_wait)
    reflex_hand.open(finger)
rospy.sleep(std_wait)
reflex_hand.command_smarts('close')
rospy.sleep(std_wait)
reflex_hand.command_smarts('open')
raw_input("== If all went well, continue to next test by pressing [Enter]\n")

###############################################################################
# Check the spherical freedom of motion
raw_input("== When ready to check spherical freedom of movement, press [Enter]\n")
rospy.sleep(std_wait)
reflex_hand.set_spherical()
rospy.sleep(std_wait)
for finger in range(3):
    reflex_hand.move_finger(finger, 2)
    rospy.sleep(std_wait)    
reflex_hand.command_smarts('open')
raw_input("== If all went well, continue to next test by pressing [Enter]\n")

###############################################################################
# Check the pinch freedom of motion
raw_input("== When ready to check pinch freedom of movement, press [Enter]\n")
rospy.sleep(std_wait)
reflex_hand.set_pinch()
for finger in range(2):
    rospy.sleep(std_wait)
    reflex_hand.close(finger)
    rospy.sleep(std_wait)
    reflex_hand.open(finger)
rospy.sleep(std_wait)
for finger in range(2):
    reflex_hand.move_finger(finger, 2)
rospy.sleep(std_wait)
reflex_hand.command_smarts('open')
rospy.sleep(std_wait)
reflex_hand.set_cylindrical()
raw_input("== If all went well, continue to next test by pressing [Enter]\n")

###############################################################################
# Check that the tactile sensors are working
while not rospy.is_shutdown():
    raw_input("== When ready to test tactile sensors, press [Enter]\n")
    zero_tactile()
    rospy.sleep(std_wait)
    res = reflex_hand.command_smarts('guarded_move')
    rospy.sleep(std_wait)
    reflex_hand.command_smarts('open')
    command = raw_input("== To rerun test, press [Enter]. To move on, press [q][Enter]\n")
    if len(command) > 0 and (command[0] == 'q' or command[0] == 'Q'):
        break

###############################################################################
# Recommend checking all tactile sensors in the visualizer
raw_input("== The easiest way to check all tactile sensors is to check by hand with \n\
== the visualizer. Press [Enter] to exit, the visualizer is run separately ")