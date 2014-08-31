#!/usr/bin/env python

###########################################################
# 
# Services for basic reflex_base commands
#
# Eric Schneider
# 
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
		rospy.loginfo("reflex_base:CommandHandService: Calling ReFlex service...")
		if self.locked:
			rospy.loginfo("reflex_base:CommandHandService: Service is locked at the moment (in use), try again later")
			return (0, -1)
		else:
			self.locked = True
			rospy.loginfo("reflex_base:CommandHandService: The requested action %s is about to run", req.action)
			start_time = rospy.Time.now()
			# self.obj.command_base(req.speed, *req.action.split(' '))
			self.obj.command_base(1.0, *req.action.split(' '))
			end_time = rospy.Time.now()
			self.locked = False
# TODO: Use more in-depth return statements than 1 and 0
			return (1, end_time-start_time)

class MoveFingerService:
	def __init__(self, obj):
		self.obj = obj
		self.locked = False

	def __call__(self, req):
		rospy.loginfo("reflex_base:MoveFingerService: Calling ReFlex service...")
		if self.locked:
			rospy.loginfo("reflex_base:MoveFingerService: Service is locked at the moment (in use), try again later")
			return (0, -1)
		else:
			self.locked = True
			rospy.loginfo("reflex_base:MoveFingerService: reflex_f%d is about to try to move to %f radians", req.finger_index+1, req.goal_pos)
			start_time = rospy.Time.now()
			# self.obj.move_finger(req.finger_index, req.goal_pos, req.speed)
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
		rospy.loginfo("reflex_base:MovePreshapeService: Calling ReFlex service...")
		if self.locked:
			rospy.loginfo("reflex_base:MovePreshapeService: Service is locked at the moment (in use), try again later")
			return (0, -1)
		else:
			self.locked = True
			rospy.loginfo("reflex_base:MovePreshapeService: preshape is about to try to move to %f radians", req.goal_pos)
			start_time = rospy.Time.now()
			# self.obj.move_preshape(req.goal_pos, req.speed)
			self.obj.move_preshape(req.goal_pos, 1.0)
			end_time = rospy.Time.now()
			self.locked = False
# TODO: Use more in-depth return statements than 1 and 0
			return (1, end_time-start_time)

class StatusDumpService:
	def __init__(self, obj):
		self.obj = obj

	def __call__(self, req):
		rospy.loginfo("reflex_base:StatusDumpService: Dumping hand data ====================>\n\
self.working: %s\nself.control_mode: %s\nself.cmd_spool: %s\nself.hand: %s"\
						, str(self.obj.working), str(self.obj.control_mode), str(self.obj.cmd_spool), str(self.obj.hand))
		self.obj.get_subscriptions();

class KillService:
	def __init__(self, obj):
		self.obj = obj
	def __call__(self, req):
		rospy.loginfo("reflex_base:KillService: Setting all fingers to working = False and commanding 'hold'")
		self.obj.working = [False, False, False]
		self.obj.command_base(1.0, 'hold')

class CommandSmartService:
	def __init__(self, obj):
		self.obj = obj
		self.locked = False

	def __call__(self, req):
		rospy.loginfo("reflex_base:CommandSmartService: Calling ReFlex service...")
		if self.locked:
			rospy.loginfo("reflex_base:CommandSmartService: Service is locked at the moment (in use), try again later")
			return (0, -1)
		else:
			self.locked = True
			rospy.loginfo("reflex_base:CommandSmartService: The requested action %s is about to run", req.action)
			start_time = rospy.Time.now()
			# self.obj.command_smarts(req.speed, *req.action.split(' '))
			self.obj.command_smarts(1.0, *req.action.split(' '))
			end_time = rospy.Time.now()
			self.locked = False
# TODO: Use more in-depth return statements than 1 and 0
			return (1, end_time-start_time)