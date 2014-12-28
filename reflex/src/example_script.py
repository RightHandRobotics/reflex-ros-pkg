#!/usr/bin/env python

# Example code for a script using the ReFlex hand
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

# Zeroing the tactile data is necessary if you wish to use a mode
# that employs tactile data (like guarded_move) because the
# sensors drift over time
zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)

##################################################################
rospy.loginfo("EXAMPLE -- Entering first command example")
rospy.sleep(2)

# Example of full-hand command using the command_smarts interface
reflex_hand.command_smarts(1, 'close')

# Note: this message isn't printed until the command completes
rospy.loginfo("EXAMPLE -- Finished first hand command example")
rospy.sleep(2)

##################################################################
rospy.loginfo("EXAMPLE -- Entering individual command example")
rospy.sleep(2)

# This is an example of using the functions available to the hand,
# not just the command_smarts interface. Opens each finger in turn
for finger in range(3):
    reflex_hand.open(finger)
    # This sleep is for visual effect
    rospy.sleep(2)

rospy.loginfo("EXAMPLE -- Finished individual command example")
rospy.sleep(2)

##################################################################
rospy.loginfo("EXAMPLE -- Entering example using tactile")
# Calling zero_tactile service zeros tactile sensors before a grasp
zero_tactile()
rospy.sleep(2)

# Example of a full-hand command using the command_smarts interface
reflex_hand.command_smarts(1, 'guarded_move')
reflex_hand.command_smarts(1, 'open')

rospy.loginfo("EXAMPLE -- Finished example using tactile")
rospy.sleep(2)

# NOTE: If the hand isn't doing guarded_move as expected, check
# visualizer to see if the hand thinks the fingers are in contact

##################################################################
rospy.loginfo("EXAMPLE -- Entering preshape control example")
rospy.sleep(2)

# Example of setting the preshape joint (angle of paired fingers)
# You can't move the fingers while adjusting the preshape joint
# Note: hand moves all the way to its goal before moving on
reflex_hand.set_cylindrical(1)
rospy.sleep(1)
reflex_hand.set_pinch(1)
rospy.sleep(1)
reflex_hand.set_cylindrical(1)

rospy.loginfo("EXAMPLE -- Leaving preshape control example")
rospy.sleep(2)

##################################################################
rospy.loginfo("EXAMPLE -- Entering position control example")
rospy.sleep(2)

# This is an example of setting all the fingers to specific angles
reflex_hand.move_finger(0, 1.0)
reflex_hand.move_finger(1, 2.0)
reflex_hand.move_finger(2, 3.0)

rospy.loginfo("EXAMPLE -- Leaving position control example")
rospy.sleep(2)

##################################################################
rospy.loginfo("EXAMPLE -- Entering ceaseless example")
rospy.loginfo("\tCeaseless example shows avoid_contact. To")
rospy.loginfo("\tmake a finger open, just apply pressure")
# Calling the zero_tactile service proxy zeros tactile sensors
# before the avoid_contact function uses the data
zero_tactile()
rospy.sleep(2)

# This is an example of a CEASELESS move. The modes with no obvious
# end state: dither, maintain_contact, avoid_contact, and hold, are
# set to working = False. You're on your own for deciding how long
# to maintain these states before giving another command
# More info: http://www.righthandrobotics.com/doc:reflex:software
reflex_hand.command_smarts(1, 'avoid_contact')
while not rospy.is_shutdown() and any(reflex_hand.working):
    rospy.sleep(0.1)
# Because this mode is ceaseless, reflex_hand.working will be false
# and this print statement should occur immediately
rospy.loginfo("EXAMPLE -- No longer in the while loop")
rospy.loginfo("EXAMPLE -- DONE with example script code")

rospy.spin()
