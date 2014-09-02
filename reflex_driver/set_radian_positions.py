#!/usr/bin/env python
import roslib; roslib.load_manifest('reflex_driver')
import rospy, sys
from reflex_msgs.msg import RawServoPositions
from reflex_msgs.msg import RadianServoPositions

if __name__ == '__main__':
  rospy.init_node('set_raw_positions')
  args = rospy.myargv()
  if len(args) != 5:
    print(" found %d args" % len(args))
    print("usage: set_radian_positions POS0 POS1 POS2 POS3")
    print("  where POSx is in radians, and is interpreted with reference")
    print("  to the zero position found in yaml/finger_calibrate.yaml")
    sys.exit(1)
  
  srp_pub = rospy.Publisher('set_reflex_hand', RadianServoPositions)

  rospy.sleep(0.3)
  pos_list = [float(args[1]), float(args[2]), float(args[3]), float(args[4])]
  srp_pub.publish(RadianServoPositions(pos_list))
  rospy.sleep(0.1)
