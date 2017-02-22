#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

import reflex_msgs.msg
import reflex_msgs.srv
import finger
from reflex_hand import ReflexHand
from reflex_takktile_hand import ReflexTakktileHand

def main():
    rospy.sleep(2.0)  # To allow services and parameters to load
    rospy.loginfo("Hello from hand 1!")
    hand1 = ReflexTakktileHand("hand1", "1")
    rospy.sleep(0.5)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        hand1._publish_motor_commands()
        if (rospy.get_rostime().secs > (hand1.latest_update.secs + hand1.comms_timeout)):
            rospy.logfatal('Hand 1 going down, no ethernet data for %d seconds', hand1.comms_timeout)
            rospy.signal_shutdown('Comms timeout')
        r.sleep()

if __name__ == '__main__':
    main()