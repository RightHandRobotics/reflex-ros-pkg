#!/usr/bin/env python

##########################################################
# 
# This extends the reflex_smarts ReFlex class and adds a series
# of preset moves
# 
# Eric Schneider
# 
########################################################## 
# TODO: Setting servo speed is implemented here but not in firmware. Waiting on that

import rospy
from math import floor
from copy import deepcopy
import sys

from std_msgs.msg import Float64

from reflex_base_services import *

# interface
# 1 arguments 		= apply to whole hand
# 3 arguments 		= apply to each finger
# 2,4,6 arguments 	= apply to each (finger, command) pair

# called on per-finger or full-hand basis
FINGER_COMMANDS = ['open', 'preshape_probe', 'tighten', 'loosen']
# called only on full-hand basis
HAND_COMMANDS = ['cylinder', 'spherical', 'pinch', 'fingerwalk', 'align_all', 'dof']

# position parameters
ROT_CYL = 0.1		# radians rotation 
ROT_SPH = 1.1		# radians rotation
ROT_PINCH = 2.3 	# radians rotation

OPEN = 0			# radians tendon spool
PROBE_POS = 0.8		# radians tendon spool
DOF_POS = 1.5
HOW_HARDER = 0.4	# radians step size for tightening and loosening

class ReFlex_Smarts(ReFlex):
	def __init__(self):
		super(ReFlex_Smarts, self).__init__()

	# Parses modes and turns them into control_modes for control_loop
	def __command_smart_finger(self, finger, mode):
		i = self.FINGER_MAP[finger]		
		if mode == 'open':
			self.working[i] = True
			self.open(i, self.SERVO_SPEED_MAX)
		elif mode == 'preshape_probe':
			self.working[i] = True
			self.preshape_probe(i, self.SERVO_SPEED_MAX)
		elif mode == 'tighten':
			self.working[i] = True
			self.tighten(i, self.SERVO_SPEED_MAX/2.0)
		elif mode == 'loosen':
			self.working[i] = True
			self.loosen(i, self.SERVO_SPEED_MAX/2.0)
		else:
			rospy.loginfo("reflex_smarts:__command_smart_finger: received an unknown finger command: %s", mode)

	def command_smarts(self, speed, *args):
		# 1 arg = hand command
		# 3 arg = finger commands
		# 2, 4, 6 arg = [finger, mode, (finger, mode, (finger, mode))]

		if len(args) == 1:
			if args[0] in HAND_COMMANDS:
				if args[0] == 'cylinder': 	self.set_cylindrical(speed)
				if args[0] == 'spherical': 	self.set_spherical(speed)
				if args[0] == 'pinch': 		self.set_pinch(speed)
				if args[0] == 'dof':		self.dof_tour(speed)
				if args[0] == 'fingerwalk':	self.fingerwalk(speed)
				if args[0] == 'align_all':	self.align_all(speed)
			else:
				self.__command_smart_finger('f1', args[0])
				self.__command_smart_finger('f2', args[0])
				self.__command_smart_finger('f3', args[0])
				self.servo_speed = [speed, speed, speed]
		elif len(args) == 3:
			self.__command_smart_finger('f1', args[0])
			self.__command_smart_finger('f2', args[1])
			self.__command_smart_finger('f3', args[2])
			self.servo_speed = [speed, speed, speed]
		elif len(args) in [2,4,6]:
			fingers = [args[2*j] for j in range(len(args)/2)]
			modes = [args[2*(j+1)-1] for j in range(len(args)/2)]

			for j in range(len(fingers)):
				self.__command_smart_finger(fingers[j], modes[j])
				self.servo_speed[j] = speed
		else:
			rospy.loginfo("reflex_smarts:command_smarts: did not recognize the input, was given %s", args)
		
		rospy.loginfo("reflex_smarts:command_smarts: commanded the fingers, self.working = %s, self.control_mode: %s"\
						 , str(self.working), str(self.control_mode))

		while any(self.working) and not rospy.is_shutdown():
			rospy.sleep(0.01)
		return

	def open(self, finger_index, speed):
		rospy.loginfo("reflex_smarts:open: Opening finger %d", finger_index+1)
		self.move_finger(finger_index, self.TENDON_MIN, speed)

	def preshape_probe(self, finger_index, speed):
		rospy.loginfo("reflex_smarts:preshape_probe: Setting finger %d to preshape probe position", finger_index+1)
		self.move_finger(finger_index, PROBE_POS, speed)

	def tighten(self, finger_index, speed):
		rospy.loginfo("reflex_smarts:tighten: Tightening finger %d", finger_index+1)
		self.move_finger(finger_index, self.hand.finger[finger_index].spool + HOW_HARDER, speed)

	def loosen(self, finger_index, speed):
		rospy.loginfo("reflex_smarts:loosen: Tightening finger %d", finger_index+1)
		self.move_finger(finger_index, self.hand.finger[finger_index].spool - HOW_HARDER, speed)

	def set_cylindrical(self, speed):
		rospy.loginfo("reflex_smarts:set_cylindrical: Going to cylindrical pose")
		self.move_preshape(ROT_CYL, speed)

	def set_spherical(self, speed):
		rospy.loginfo("reflex_smarts:set_spherical: Going to spherical pose")
		self.move_preshape(ROT_SPH, speed)

	def set_pinch(self, speed):
		rospy.loginfo("reflex_smarts:set_pinch: Going to pinch pose")
		self.move_preshape(ROT_PINCH, speed)

	# Runs the hand through it's range of motions, using both finger positions and preshape joint
	def dof_tour(self, speed):
		rospy.loginfo("reflex_smarts:dof_tour: Exploring hand DOF...")
		self.move_preshape(ROT_CYL, speed)
		for i in range(3):
			self.move_finger(i, DOF_POS, speed)
			rospy.sleep(1.5)
			self.open(i, speed)
			rospy.sleep(1)
		for pos in [ROT_CYL, ROT_SPH, ROT_PINCH, ROT_CYL]:
			self.move_preshape(pos, speed)
			rospy.sleep(1.5)
		return

	# Performs a preset routine to tighten fingers and walk object into a solid grip
	def fingerwalk(self, speed, in_step = 0.6, out_step = 1.0):
		rospy.loginfo("reflex_smarts:fingerwalk: Starting fingerwalk...")
		current_state = deepcopy(self.hand)
		# self.latching = True
		counter = 0		# fail-safe to prevent overheating motors

		if len(self.hand_hist) > 0:
			while not any([self.hand_hist[0].palm.contact[i] for i in range(11)])\
						and not all([self.finger_full_contact(i) for i in range(3)])\
							and (counter < floor(1.8/in_step))\
								and not rospy.is_shutdown():
				
				for i in range(3):
					self.move_finger(i, current_state.finger[i].spool+in_step, speed)
					current_state.finger[i].spool += in_step
				while any(self.working) and not rospy.is_shutdown():	rospy.sleep(0.01)
				
				self.move_finger(0, current_state.finger[0].spool-out_step, speed)
				while any(self.working) and not rospy.is_shutdown():	rospy.sleep(0.01)
				self.move_finger(0, current_state.finger[0].spool, speed)
				while any(self.working) and not rospy.is_shutdown():	rospy.sleep(0.01)

				self.move_finger(1, current_state.finger[0].spool-out_step, speed)
				while any(self.working) and not rospy.is_shutdown():	rospy.sleep(0.01)
				self.move_finger(1, current_state.finger[0].spool, speed)
				while any(self.working) and not rospy.is_shutdown():	rospy.sleep(0.01)	
				
				counter += 1
				rospy.loginfo("reflex_smarts:fingerwalk: Have completed %d fingerwalk cycles", counter)

			# self.latching = False
			return
		else:
			rospy.loginfo("reflex_smarts:fingerwalk: There is no Hand data being read, cannot do fingerwalk")
			return

	# Finds the average of the three finger spool values and sets them all to that
	def align_all(self, speed):
		avg_spool = sum([self.hand.finger[i].spool for i in range(3)])/3.0
		rospy.loginfo("reflex_smarts:align_all: Setting all fingers to avg spool setting: %f", avg_spool)
		for i in range(3):
			self.move_finger(i, avg_spool, speed)



if __name__ == '__main__':
	rospy.init_node('ReflexServiceNode')
	reflex_hand = ReFlex_Smarts()

	sh1 = CommandHandService(reflex_hand)
	s1  = "/reflex/command_base"
	rospy.loginfo("reflex:__main__: Advertising the %s service", s1)
	s1 = rospy.Service(s1, CommandHand, sh1)

	sh2 = MoveFingerService(reflex_hand)
	s2  = "/reflex/move_finger"
	rospy.loginfo("reflex:__main__: Advertising the %s service", s2)
	s2 = rospy.Service(s2, MoveFinger, sh2)

	sh3 = MovePreshapeService(reflex_hand)
	s3  = "/reflex/move_preshape"
	rospy.loginfo("reflex:__main__: Advertising the %s service", s3)
	s3 = rospy.Service(s3, MovePreshape, sh3)

	sh4 = StatusDumpService(reflex_hand)
	s4  = "/reflex/status_dump"
	rospy.loginfo("reflex:__main__: Advertising the %s service", s4)
	s4 = rospy.Service(s4, Empty, sh4)

	sh5 = KillService(reflex_hand)
	s5  = "/reflex/kill_current"
	rospy.loginfo("reflex:__main__: Advertising the %s service", s5)
	s5 = rospy.Service(s5, Empty, sh5)

	sh6 = CommandSmartService(reflex_hand)
	s6  = "/reflex/command_smarts"
	rospy.loginfo("reflex_smarts:__main__: Advertising the %s service", s6)
	s6 = rospy.Service(s6, CommandHand, sh6)

	r_fast = rospy.Rate(50)
	r_slow = rospy.Rate(1)
	while not rospy.is_shutdown():
		if reflex_hand.hand_publishing:
			r_slow.sleep()
		else:
			reflex_hand._ReFlex__control_loop()
			r_fast.sleep()
