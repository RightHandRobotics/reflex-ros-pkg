#!/usr/bin/env python
import roslib; roslib.load_manifest('reflex_hand')
import rospy, sys
from reflex_hand.msg import RawServoPositions
from reflex_hand.msg import RadianServoPositions

if __name__ == '__main__':
  rospy.init_node('set_raw_positions')
  args = rospy.myargv()
  if len(args) != 5:
    print(" found %d args" % len(args))
    print("usage: set_raw_positions POS0 POS1 POS2 POS3")
    print("  where POSx is going straight to a dynamixel, so it should be")
    print("  in [0, 4095]")
    sys.exit(1)
  # srp_pub = rospy.Publisher('set_reflex_raw', RawServoPositions)
  srp_pub = rospy.Publisher('set_reflex_hand', RadianServoPositions)
  rospy.sleep(0.3) # terrible terrible... but hey this is a test program...
  # pos_list = [int(args[1]), int(args[2]), int(args[3]), int(args[4])]
  pos_list = [float(args[1]), float(args[2]), float(args[3]), float(args[4])]
  # srp_pub.publish(RawServoPositions(pos_list))
  srp_pub.publish(RadianServoPositions(pos_list))
  rospy.sleep(0.1)
