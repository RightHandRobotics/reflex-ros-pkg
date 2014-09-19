#!/usr/bin/env python

###########################################################
# 
# Exposes basic functionality of the hand in a ReFlex class
# The reflex package is designed to be used in two ways
# First, you can instantiate a ReFlex class and use it's methods/variables. This is intended for scripts
# Second, you can run reflex_base.py and use the services advertised to run commands. This is for prototyping
#
# Eric Schneider
# 
###########################################################
# TODO: Setting servo speed is implemented here but not in firmware.
# Pass speed to the hand in __control_loop when implemented

import rospy
import numpy as np
from copy import deepcopy
import sys

from std_msgs.msg import Float64
from std_srvs.srv import Empty
from reflex_base_services import *
from reflex_msgs.msg import Hand, Event, RadianServoPositions
from reflex_msgs.srv import CommandHand, MoveFinger, MovePreshape

# methods to control individual fingers
#	goto
#	guarded_move
#	solid_contact
#	avoid_contact
#	maintain_contact
#	dither
#	hold
#	loose

# methods to preshape
#	None - done in move_preshape, not control_loop

# methods that provide feedback about the state of the hand:
#	hand_in_contact()
# 	finger_in_contact(fingerID)
#	finger_in_fullcontact(fingerID)

# methods to control the full hand
#	None

class ReFlex(object):
	def __init__(self):
		super(ReFlex, self).__init__()

		# basic commands
		self.BASE_FINGER_COMMANDS = ['goto', 'guarded_move', 'guarded_move_fast', 'solid_contact', 'avoid_contact', 'maintain_contact', 'dither', 'hold', 'loose']
		self.FINGER_MAP = {'f1':0,'f2':1,'f3':2}

		# motion parameters
		self.FINGER_STEP = 0.05 						# radians / 0.01 second
		self.FINGER_STEP_LARGE = 0.15 					# radians / 0.01 second
		self.SERVO_SPEED_MIN = 0.0 						# radians / second
		self.SERVO_SPEED_MAX = 3.0 						# radians / second
		self.TENDON_MIN = 0.0							# max opening (radians)
		self.TENDON_MAX = 3.5							# max closure (radians)
		self.PRESHAPE_MIN = 0.0							# max opening (radians)
		self.PRESHAPE_MAX = 1.6							# max closure (radians)

		# control loop logic parameters for safety checking
		self.ARRIVAL_ERROR = 0.05						# error at which the control loop considers position control finished
		self.BLOCKED_ERROR = 0.05 						# if the finger hasn't moved this far in hand_hist steps, it's blocked

		# Initialize logic variables
		self.control_mode = ['goto', 'goto', 'goto']		# control mode for each finger
		self.working = [False, False, False]				# completion criteria (service returns when all false)
		topic = "/reflex/finger_events"						# used to publish self.working events
		rospy.loginfo("ReFlex class is publishing the %s topic", topic)
		self.event_pub = rospy.Publisher(topic, Event, queue_size = 10)
		self.event_reason = [-1, -1, -1];
		self.cmd_spool = np.array([-1.0, -1.0, -1.0]) 		# finger commanded position, radians spool rotation
		self.cmd_spool_old = deepcopy(self.cmd_spool)		# Previous cmd position. If current cmd position matches, give no command
		self.servo_speed = np.array([-1.0, -1.0, -1.0]) 	# finger commanded position, radians spool rotation

		# Set up motor publishers
		self.actuator_pub = rospy.Publisher('/set_reflex_hand', RadianServoPositions)

		# Subscribe to sensor information
		self.hand_publishing = True
		self.hand = Hand()
		self.latching = False
		self.hand_hist = []									# history of incoming hand data, bounded in __hand_callback
		self.HIST_INTERVAL = 1								# every self.HIST_INTERVAL the hand_hist variable is updated
															# this number should probably increase when /reflex_hand speeds up
		self.HIST_COUNTER = 0 								# used to count how many times the control loop has been run
		self.HISTORY_LENGTH = 20							# how many snapshots of old data to keep for checking motor stall, etc

		self.SUBSCRIBE_TIME = 5 							# time (seconds) that code will wait for a subscription
		topic = "/reflex_hand"
		rospy.loginfo("ReFlex class is subscribing to topic %s", topic)
		rospy.Subscriber(topic, Hand, self.__hand_callback)
		time_waiting = 0.0
		while len(self.hand_hist) < 2 and time_waiting < self.SUBSCRIBE_TIME and not rospy.is_shutdown():
			rospy.sleep(0.01)
			time_waiting += 0.01
		self.hand_publishing = did_subscribe_succeed(time_waiting, topic)
		if not self.hand_publishing:
			rospy.logwarn("\tCheck that your hand is plugged in to power and the ethernet port")

		rospy.loginfo("Reflex class is initialized\n...running...")


	def __hand_callback(self, msg):
		"""updates the local cache of the hand state whenever a message arrives from reflex_driver_node"""
		self.hand = deepcopy(msg)
		if (self.HIST_COUNTER % self.HIST_INTERVAL) == 0:
			self.hand_hist.append(self.hand)

		# Allow latching
		# if self.latching and len(self.hand_hist) > 1:
		# 	for i in range(len(self.hand.finger)):
		# 		for j in range(len(self.hand.finger[i].contact)):
		# 			if self.hand_hist[-2].finger[i].contact[j]:
		# 				self.hand_hist[-1].finger[i].contact[j] = True
		# 	for j in range(len(self.hand.palm.contact)):
		# 		if self.hand_hist[-2].palm.contact[j]:
		# 			self.hand_hist[-1].palm.contact[j] = True

		# Bounds the hist list to N elements
		while len(self.hand_hist) >= self.HISTORY_LENGTH:
			self.hand_hist.pop(0)

		self.__control_loop()


	def __control_loop(self):
		publishing_error = "reflex_base:__control_loop: tactile_publishing = %s and joints_publishing = %s, both must be true to use " % \
								(self.hand.tactile_publishing, self.hand.joints_publishing)
		working_state = deepcopy(self.working)
		self.event_reason = [-1, -1, -1];

		# each finger has a control mode, which corresponds to some basic logic
		cmd_change = []
		pos_error = []
		for i in range(3):
			# position control mode
			motor_error = self.cmd_spool[i] - self.hand.finger[i].spool
			if self.control_mode[i] == 'goto':
				if not self.hand.joints_publishing:
					self.working[i] = False
					self.event_reason[i] = 1
				elif self.working[i] and (abs(motor_error) < self.ARRIVAL_ERROR or\
						abs(self.hand_hist[0].finger[i].spool - self.hand_hist[-1].finger[i].spool) < self.BLOCKED_ERROR):
					self.working[i] = False
					self.event_reason[i] = 0

			# guarded modes
			elif self.control_mode[i] == 'guarded_move':		# close until either link experiences contact
				if self.hand.tactile_publishing and self.hand.joints_publishing:
					if self.finger_in_contact(i) or (self.hand.finger[i].spool >= self.TENDON_MAX):
						self.working[i] = False
						self.event_reason[i] = 0
					elif self.working[i]:
						self.cmd_spool[i] = self.hand.finger[i].spool+self.FINGER_STEP
				else:
					rospy.loginfo(publishing_error + "guarded_move")
					self.working[i] = False
					self.event_reason[i] = 1
					
			# guarded modes
			elif self.control_mode[i] == 'guarded_move_fast':		# close until either link experiences contact
				if self.hand.tactile_publishing and self.hand.joints_publishing:
					if self.finger_in_contact(i) or (self.hand.finger[i].spool >= self.TENDON_MAX):
						self.working[i] = False
						self.event_reason[i] = 0
					elif self.working[i]:
						self.cmd_spool[i] = self.hand.finger[i].spool+self.FINGER_STEP_LARGE
				else:
					rospy.loginfo(publishing_error + "guarded_move_fast")
					self.working[i] = False
					self.event_reason[i] = 1
			
			elif self.control_mode[i] == 'solid_contact':		# close until both links experience contact
				if self.hand.tactile_publishing and self.hand.joints_publishing:
					if self.finger_full_contact(i) or (self.hand.finger[i].spool >= self.TENDON_MAX):
						self.working[i] = False
						self.event_reason[i] = 0
					elif self.working[i]:
						self.cmd_spool[i] = self.hand.finger[i].spool+self.FINGER_STEP
				else:
					rospy.loginfo(publishing_error + "solid_contact")
					self.working[i] = False
					self.event_reason[i] = 1

			# ceaseless modes
			elif self.control_mode[i] == 'dither':				# if in contact, loosen; if no contact, tighten
				if self.hand.tactile_publishing and self.hand.joints_publishing:
					if self.finger_in_contact(i) and (self.hand.finger[i].spool > self.TENDON_MIN):
						self.cmd_spool[i] -= self.FINGER_STEP
					elif (self.hand.finger[i].spool < self.TENDON_MAX):
						self.cmd_spool[i] += self.FINGER_STEP
				else:
					rospy.loginfo(publishing_error + "dither")

			elif self.control_mode[i] == 'avoid_contact':		# open finger if contact
				if self.hand.tactile_publishing and self.hand.joints_publishing:
					if self.finger_in_contact(i) and (self.hand.finger[i].spool > self.TENDON_MIN):
						self.cmd_spool[i] = self.hand.finger[i].spool-self.FINGER_STEP
				else:
					rospy.loginfo(publishing_error + "avoid_contact")

			elif self.control_mode[i] == 'maintain_contact':	# if no contact, move finger in
				if self.hand.tactile_publishing and self.hand.joints_publishing:
					if not self.finger_in_contact(i) and (self.hand.finger[i].spool < self.TENDON_MAX):
						self.cmd_spool[i] = self.hand.finger[i].spool+self.FINGER_STEP
				else:
					rospy.loginfo(publishing_error + "maintain_contact")
			
			elif self.control_mode[i] == 'hold':
				pass

			elif self.control_mode[i] == 'loose':
				rospy.logwarn('loose: not yet implemented')
				# TODO: Find a way to make the Dynamixels go loose and exert no force, make it happen here
				pass

			else:
				rospy.loginfo("reflex_base:__control_loop: Found unrecognized control_mode: %s", self.control_mode[i])

			# if self.working[i] and abs(self.hand_hist[0].finger[i].spool - self.hand_hist[-1].finger[i].spool) < self.BLOCKED_ERROR:
				# rospy.logwarn("FINGER %d STOPPED ON BLOCKED ERROR", i+1);
				# self.working[i] = False;

			# If an action has been completed, publish an event
			if (working_state[i] == True) and (self.working[i] == False):
				self.event_pub.publish("Finger %d has completed '%s' action"%(i, self.control_mode[i]),\
										self.control_mode[i], i, self.event_reason[i], rospy.Time.now())

			# execute finger control 
			self.cmd_spool[i] = min(max(self.cmd_spool[i], self.TENDON_MIN), self.TENDON_MAX)
			self.servo_speed[i] = min(max(self.servo_speed[i], self.SERVO_SPEED_MIN), self.SERVO_SPEED_MAX)
			cmd_change.append(self.cmd_spool[i] != self.cmd_spool_old[i])
			pos_error.append(motor_error > self.ARRIVAL_ERROR and self.working[i])
			self.cmd_spool_old[i] = deepcopy(self.cmd_spool[i])

			if self.working[i] and (self.control_mode[i] == 'guarded_move' or self.control_mode[i] == 'solid_contact' or self.control_mode[i] == 'guarded_move_fast'):
				rospy.loginfo("Finger %d\tControl mode: %s\tStill working: True", i+1, self.control_mode[i])

		if sum(cmd_change) or sum(pos_error):
			pos_list = [self.cmd_spool[0], self.cmd_spool[1], self.cmd_spool[2], min(max(self.hand.palm.preshape, self.PRESHAPE_MIN), self.PRESHAPE_MAX)]
			self.actuator_pub.publish(RadianServoPositions(pos_list))


	# Commands the preshape joint to move to a certain position
	def move_preshape(self, goal_pos, speed):
		if not self.hand.palm.preshape == goal_pos:
			cmd = min(max(goal_pos, self.PRESHAPE_MIN), self.PRESHAPE_MAX)
			speed = min(max(speed, self.SERVO_SPEED_MIN), self.SERVO_SPEED_MAX)
			pos_list = [self.hand.finger[0].spool, self.hand.finger[1].spool, self.hand.finger[2].spool, cmd]
			self.actuator_pub.publish(RadianServoPositions(pos_list))

			motor_error = goal_pos - self.hand.palm.preshape
			if not self.hand.joints_publishing:
				return
			else:
				while (abs(motor_error) > self.ARRIVAL_ERROR)\
						and (abs(self.hand_hist[0].palm.preshape - self.hand_hist[-1].palm.preshape) > self.BLOCKED_ERROR)\
							and not rospy.is_shutdown():
					motor_error = goal_pos - self.hand.palm.preshape
					rospy.sleep(0.01)
			return

	
	# Commands a finger to move to a certain position
	def move_finger(self, finger_index, goal_pos, speed):
		self.control_mode[finger_index] = 'goto'
		self.working[finger_index] = True
		self.cmd_spool[finger_index] = goal_pos
		self.servo_speed[finger_index] = speed
		return


	# Parses modes and turns them into control_mode for control_loop
	def __command_base_finger(self, finger, mode):
		i = self.FINGER_MAP[finger]		
		if mode == 'guarded_move':
			self.working[i] = True
			self.control_mode[i] = 'guarded_move'
		if mode == 'guarded_move_fast':
			self.working[i] = True
			self.control_mode[i] = 'guarded_move_fast'
		elif mode == 'solid_contact':
			self.working[i] = True
			self.control_mode[i] = 'solid_contact'
		elif mode == 'avoid_contact':
			self.working[i] = False
			self.control_mode[i] = 'avoid_contact'
		elif mode == 'maintain_contact':
			self.working[i] = False
			self.control_mode[i] = 'maintain_contact'
		elif mode == 'dither':
			self.working[i] = False
			self.control_mode[i] = 'dither'
		elif mode == 'hold':
			self.working[i] = False
			self.control_mode[i] = 'hold'
		elif mode == 'loose':
			self.working[i] = False
			self.control_mode[i] = 'loose'
		else:
			rospy.logwarn("reflex_base:__command_base_finger: received an unknown finger command: %s", mode)
		self.reset_hist(self.working[i], i)


	def command_base(self, speed, *args):
		# 1 arg = hand command
		# 3 arg = finger commands
		# 2, 4, 6 arg = [finger, mode, (finger, mode, (finger, mode))]
		
		flag = False
		if len(args) == 1:
			self.__command_base_finger('f1', args[0]) 
			self.__command_base_finger('f2', args[0])
			self.__command_base_finger('f3', args[0])
			self.servo_speed = [speed, speed, speed]

			if args[0] not in self.BASE_FINGER_COMMANDS:
				flag = True

		elif len(args) == 3:
			self.__command_base_finger('f1', args[0])
			self.__command_base_finger('f2', args[1])
			self.__command_base_finger('f3', args[2])
			self.servo_speed = [speed, speed, speed]

			for i in range(3):
				if args[i] not in self.BASE_FINGER_COMMANDS:
					flag = True

		elif len(args) in [2,4,6]:
			fingers = [args[2*j] for j in range(len(args)/2)]
			modes = [args[2*(j+1)-1] for j in range(len(args)/2)]
			for j in range(len(fingers)):
				self.__command_base_finger(fingers[j], modes[j])
				self.servo_speed[j] = speed

			for mode in modes:
				if mode not in self.BASE_FINGER_COMMANDS:
					flag = True
		else:
			rospy.loginfo("reflex_base:command_base: did not recognize the input, was given %s", args)
			flag = True
		
		rospy.loginfo("reflex_base:command_base: commanded the fingers, self.working = %s, self.control_mode: %s"\
						, str(self.working), str(self.control_mode))

		while any(self.working) and not rospy.is_shutdown():
			rospy.sleep(0.01)
		return flag


	# Are any of the sensors (finger or palm) in contact?
	def hand_in_contact(self):
		any_contact = False
		for i in range(len(self.hand.finger)):
			if self.finger_in_contact(i):
				any_contact = True
		if sum(self.hand.palm.contact) > 0:
			any_contact = True
		return any_contact


	# Has the finger made contact?
	def finger_in_contact(self, finger_index):
		return sum(self.hand.finger[finger_index].contact) > 0
			# TODO: Re-enable this statement when the calculated distal angle is accurate
			# or (self.hand.finger[finger_index].distal > self.distal_joint_contact(finger_index))


	# Are both links in contact?
	def finger_full_contact(self, finger_index):
		return sum(self.hand.finger[finger_index].contact[5:9]) and sum(self.hand.finger[finger_index].contact[0:5])
		# TODO: Re-enable this calculation when the statement distal angle is accurate
		# or self.hand.finger[finger_index].distal > self.distal_joint_contact(finger_index)


	# Is the distal link bent
	def distal_joint_contact(self, finger_index):
		spool_level = [self.TENDON_MIN, self.TENDON_MAX]
		contact_angle = [0.3, 0.75]							# Angle in distal link indicating contact (radians)
		
		# Fit current spool level to a linear slope
		slope = (contact_angle[1] - contact_angle[0])/(spool_level[1]-spool_level[0])
		y_intercept = (0 - spool_level[0])*slope
		return slope*self.hand.finger[finger_index].spool + y_intercept

	def reset_hist(self, flag, finger_index):
		# This resetting of the 'spool' variable is so that the finger doesn't get stuck as BLOCKED just after
		# getting commanded. It would get stuck otherwise, because before getting commanded it may
		# not have moved for a while, and would raise the BLOCKED flag
		if flag:
			for i in range(len(self.hand_hist)):
				self.hand_hist[i].finger[finger_index].spool = -1


	def guarded_move(self):
		self.__command_base('guarded_move')


	def guarded_move_fast(self):
		self.__command_base('guarded_move_fast')


	def solid_contact(self):
		self.__command_base('solid_contact')

	
	def avoid_contact(self):
		self.__command_base('avoid_contact')


	def maintain_contact(self):
		self.__command_base('maintain_contact')


	def dither(self):
		self.__command_base('dither')


	def hold(self):
		self.__command_base('hold')


	def loose(self):
		self.__command_base('loose')


	def get_subscriptions(self):
		rospy.loginfo("hand_publishing = %s", self.hand_publishing)


def did_subscribe_succeed(time_waiting, topic):
	if time_waiting >= 3:
		rospy.logwarn("ReFlex class failed to subscribe to topic %s", topic)
		return False
	else:
		rospy.loginfo("ReFlex class subscribed to topic %s", topic)
		return True


if __name__ == '__main__':
	rospy.init_node('ReflexServiceNode')
	rospy.sleep(0.5)
	reflex_hand = ReFlex()

	sh1 = CommandHandService(reflex_hand)
	s1  = "/reflex/command_base"
	rospy.loginfo("reflex_base:__main__: Advertising the %s service", s1)
	s1 = rospy.Service(s1, CommandHand, sh1)

	sh2 = MoveFingerService(reflex_hand)
	s2  = "/reflex/move_finger"
	rospy.loginfo("reflex_base:__main__: Advertising the %s service", s2)
	s2 = rospy.Service(s2, MoveFinger, sh2)

	sh3 = MovePreshapeService(reflex_hand)
	s3  = "/reflex/move_preshape"
	rospy.loginfo("reflex_base:__main__: Advertising the %s service", s3)
	s3 = rospy.Service(s3, MovePreshape, sh3)

	sh4 = StatusDumpService(reflex_hand)
	s4  = "/reflex/status_dump"
	rospy.loginfo("reflex_base:__main__: Advertising the %s service", s4)
	s4 = rospy.Service(s4, Empty, sh4)

	sh5 = KillService(reflex_hand)
	s5  = "/reflex/kill_current"
	rospy.loginfo("reflex_base:__main__: Advertising the %s service", s5)
	s5 = rospy.Service(s5, Empty, sh5)

	r_fast = rospy.Rate(50)
	r_slow = rospy.Rate(1)
	while not rospy.is_shutdown():
		if reflex_hand.hand_publishing:
			r_slow.sleep()
		else:
			reflex_hand._ReFlex__control_loop()
			r_fast.sleep()
