#!/usr/bin/env python

#############################################################################
# Copyright 2017 Right Hand Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#############################################################################

import rospy
import numpy as np
import math
import argparse
from reflex_msgs.msg import Hand
from imu import quaternion_to_matrix
from imu import get_distal_rotation

# angle_between_palm_finger: given the palm and finger quat values, determines the smallest angle between the two imus' orientations
# float[4] palmQuat: palm quat values
# float[4] fingerQuat: finger quat values
# return float: angle between the palm and finger imus in degrees
def angle_between_palm_finger(palmQuat, fingerQuat):
    palm = quaternion_to_matrix(palmQuat)
    finger = quaternion_to_matrix(fingerQuat)
    product = np.dot(np.transpose(palm), finger)
    return (math.acos((np.trace(product) - 1)/2))/math.pi*180

def callback(data):
    global FINGER_IDX
    if FINGER_IDX not in [1, 2, 3]:
        rospy.loginfo("Finger index invalid, assuming finger 1")
        FINGER_IDX = 1
    if ANGLE_BETWEEN:
        rospy.loginfo("F%d Angle: %s", FINGER_IDX, str(angle_between_palm_finger(data.palmImu.quat, data.finger[FINGER_IDX - 1].imu.quat)))
    if ROTATION:
        rospy.loginfo("F%d Rotation: %s", FINGER_IDX, str(get_distal_rotation(data.palmImu.quat,
                                                            math.pi if FINGER_IDX==3 else data.motor[3].joint_angle,
                                                            data.finger[FINGER_IDX - 1].proximal,
                                                            data.finger[FINGER_IDX - 1].imu.quat)))

def listener():
    rospy.init_node('imu', anonymous=True)
    rospy.Subscriber("/reflex_takktile/hand_state", Hand, callback)
    rospy.spin()

#To display the angle between or the rotation of a finger [1, 2, 3], use the following command with a hand running:
# rosrun reflex rotation_display.py -a -r
if __name__ == '__main__':
    global FINGER_IDX, ANGLE_BETWEEN, ROTATION
    parser = argparse.ArgumentParser(description='Prints information about the orientation of the distal link of a finger',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--finger-idx',
                        help='Select a finger out of [1, 2, 3]',
                        type=int,
                        default=1)
    parser.add_argument('-a', '--angle-between',
                        help='Print the shortest angle between the orientations of the palm and fingertip',
                        action='store_true')
    parser.add_argument('-r', '--rotation',
                        help='Print the rotation from the proximal link to the distal link of the finger',
                        action='store_true')
    args = parser.parse_args()
    FINGER_IDX = int(args.finger_idx)
    ANGLE_BETWEEN = args.angle_between
    ROTATION = args.rotation
    listener()
