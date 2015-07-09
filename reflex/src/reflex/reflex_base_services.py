#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

from reflex_base import ReFlex
from reflex_msgs.msg import Hand
from reflex_msgs.srv import CommandHand, MoveFinger, MovePreshape


class CommandSmartService:
    def __init__(self, obj):
        self.obj = obj
        self.locked = False

    def __call__(self, req):
        rospy.loginfo("reflex_base:CommandSmartService:")
        if self.locked:
            rospy.loginfo("\tService locked at the moment (in use), try later")
            return (0, -1, -1)
        else:
            self.locked = True
            rospy.loginfo("\tRequested action %s is about to run", req.action)
            start_time = rospy.Time.now()
            flag = self.obj.command_smarts(*req.action.split(' '))
            end_time = rospy.Time.now()
            self.locked = False
# TODO: Use more in-depth return statements than 1 and 0
            if flag:
                parse_response = "ERROR: Given command was unknown"
            else:
                parse_response = "Command parsed"
            return (parse_response, 1, end_time - start_time)


class KillService:
    def __init__(self, obj):
        self.obj = obj

    def __call__(self, req):
        rospy.loginfo("reflex_base:KillService:")
        rospy.loginfo("\tSetting all fingers to working = False")
        rospy.loginfo("\tcommanding 'hold'")
        self.obj.working = [False, False, False]
        self.obj.command_base('hold')
        return []
