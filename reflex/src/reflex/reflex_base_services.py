#!/usr/bin/env python

###########################################################
# Services for basic reflex package commands
# Eric Schneider
###########################################################
# TODO: Setting servo speed is implemented here but not in firmware.
# Uncomment speed code here and in srv message when implemented

import rospy

from std_srvs.srv import Empty

from reflex_base import ReFlex
from reflex_msgs.msg import Hand
from reflex_msgs.srv import CommandHand, MoveFinger, MovePreshape


class CommandHandService:
    def __init__(self, obj):
        self.obj = obj
        self.locked = False

    def __call__(self, req):
        rospy.loginfo("reflex_base:CommandHandService:")
        if self.locked:
            rospy.loginfo("\tService locked at the moment (in use), try later")
            return (0, -1)
        else:
            self.locked = True
            rospy.loginfo("\tThe action %s is about to run", req.action)
            start_time = rospy.Time.now()
            flag = self.obj.command_base(1.0, *req.action.split(' '))
            end_time = rospy.Time.now()
            self.locked = False
# TODO: Use more in-depth return statements than 1 and 0
            if flag:
                parse_response = "ERROR: Given command was unknown"
            else:
                parse_response = "Command parsed"
            return (parse_response, 1, end_time-start_time)


class MoveFingerService:
    def __init__(self, obj):
        self.obj = obj
        self.locked = False

    def __call__(self, req):
        rospy.loginfo("reflex_base:MoveFingerService:")
        if self.locked:
            rospy.loginfo("\tService locked at the moment (in use), try later")
            return (0, -1)
        else:
            self.locked = True
            start_time = rospy.Time.now()
            if req.finger_index < 0 or req.finger_index > 2:
                rospy.logwarn("Commanded finger index %d, only [0:2] allowed",
                              req.finger_index)
            elif req.goal_pos < self.obj.TENDON_MIN\
                    or req.goal_pos > self.obj.TENDON_MAX:
                rospy.logwarn("Commanded goal pos %f radians, [%f:%f] allowed",
                              req.goal_pos,
                              self.obj.TENDON_MIN,
                              self.obj.TENDON_MAX)
            else:
                rospy.loginfo("\treflex_f%d is moving to %f radians",
                              req.finger_index+1, req.goal_pos)
                self.obj.move_finger(req.finger_index, req.goal_pos, 1.0)
            end_time = rospy.Time.now()
            self.locked = False
# TODO: Use more in-depth return statements than 1 and 0
            return (1, end_time-start_time)


class MovePreshapeService:
    def __init__(self, obj):
        self.obj = obj
        self.locked = False

    def __call__(self, req):
        rospy.loginfo("reflex_base:MovePreshapeService:")
        if self.locked:
            rospy.loginfo("\tService locked at the moment (in use), try later")
            return (0, -1)
        else:
            self.locked = True
            start_time = rospy.Time.now()

            if req.goal_pos < self.obj.PRESHAPE_MIN\
               or req.goal_pos > self.obj.PRESHAPE_MAX:
                rospy.logwarn("Commanded goal pos %f radians, [%f:%f] allowed",
                              req.goal_pos,
                              self.obj.PRESHAPE_MIN,
                              self.obj.PRESHAPE_MAX)
            else:
                rospy.loginfo("\tpreshape moving to %f radians", req.goal_pos)
                self.obj.move_preshape(req.goal_pos, 1.0)

            end_time = rospy.Time.now()
            self.locked = False
# TODO: Use more in-depth return statements than 1 and 0
            return (1, end_time-start_time)


class StatusDumpService:
    def __init__(self, obj):
        self.obj = obj

    def __call__(self, req):
        rospy.loginfo("reflex_base:StatusDumpService:")
        rospy.loginfo("Dumping hand data ====================>\n\
self.working: %s\n\
self.control_mode: %s\n\
self.cmd_spool: %s\n\
hand_hist[0].spool: %s, hand_hist[-1].spool: %s\n\
self.hand: %s",
                      str(self.obj.working),
                      str(self.obj.control_mode),
                      str([self.obj.hand_hist[0].finger[0].spool,
                           self.obj.hand_hist[0].finger[1].spool,
                           self.obj.hand_hist[0].finger[2].spool]),
                      str([self.obj.hand_hist[-1].finger[0].spool,
                           self.obj.hand_hist[-1].finger[1].spool,
                           self.obj.hand_hist[-1].finger[2].spool]),
                      str(self.obj.cmd_spool), str(self.obj.hand))
        self.obj.get_subscriptions()
        return []


class KillService:
    def __init__(self, obj):
        self.obj = obj

    def __call__(self, req):
        rospy.loginfo("reflex_base:KillService:")
        rospy.loginfo("\tSetting all fingers to working = False")
        rospy.loginfo("\tcommanding 'hold'")
        self.obj.working = [False, False, False]
        self.obj.command_base(1.0, 'hold')
        return []


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
