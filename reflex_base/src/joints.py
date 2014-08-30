#!/usr/bin/env python

##################################################################
#																 #
# Parse info from TakkTile sensors into higher-level information #
#																 #
##################################################################

# import roslib; roslib.load_manifest('reflex_smarts')
import rospy

import numpy as np

from dynamixel_msgs.msg import MotorStateList
from std_msgs.msg import Header
from geometry_msgs.msg import Point

# to set zeros
#	command actuators until the fingers are in the plane of the palm
#		rostopic pub /reflex/actuators reflex_msgs/Actuators "[0.1, 0.2, 0.3, 0.0]"
# 	then look at results from /reflex/proximal_encoders
#		rostopic echo /reflex/proximal encoders
#	and also the results from the motors by setting SPOOL_ZERO to [0,0,0,0]

ENCODER_ZERO = [-2.8259761333465576, 1.56159245967865, 4.833957195281982]  #[-2.825592517852783, 1.5381991863250732, 4.7856364250183105]#[-2.784175157546997, 1.4937137365341187, -1.167960524559021]
SPOOL_ZERO = [1.2, 1.5, 1.4, -0.14]
#ENCODER_ZERO = [0,0,0]
#SPOOL_ZERO = [0,0,0,0]

def Ry(theta):
	return np.matrix([[np.cos(theta),  0, np.sin(theta)],
					  [			    0, 1, 			  0],
					  [-np.sin(theta), 0, np.cos(theta)]])

def Rz(theta):
	return np.matrix([[np.cos(theta),  np.sin(theta), 0],
					 [-np.sin(theta),  np.cos(theta), 0],
					 [             0,              0, 1]]) 
	
def pack_points(pointarray):
	points = []
	for i in range(pointarray.shape[1]):
		points += [Point(pointarray[0,i],pointarray[1,i],pointarray[2,i])]
	return points

def forward_kinematics(base_joints, distal_joints):
		# forward kinematics

		# Forward Kinematics
		# hand base ---- *rotate* ---- finger1 knuckle - *base* - finger1 proximal - *distal* - finger1 distal
		#           ---- *rotate* ---- finger2 knuckle - *base* - finger2 proximal - *distal* - finger2 distal
		#           ----  (fixed) ---- finger3 knuckle - *base* - finger3 proximal - *distal* - finger3 distal
		#
		#transforms = {	'hand-f1knuckle':[[0.026,   0.0354,  0.076], ['rz', 'pi/2-j4']], 'f1base-f1prox':[[0.00825, 0,  0], ['ry', 'base[0]']], 'f1prox-f1tip':[[0.068, 0, 0.0325],['ry','distal[0]']],
		#			    'hand-f2knuckle':[[-0.026,  0.0354,  0.076], ['rz', 'pi/2+j4']], 'f2base-f2prox':[[0.00825, 0,  0], ['ry', 'base[1]']], 'f2prox-f2tip':[[0.068, 0, 0.0325],['ry','distal[0]']],
		#			    'hand-f3knuckle':[[0.00,   -0.0354,  0.076], ['rz',   '-pi/2']], 'f3base-f3prox':[[0.00825, 0,  0], ['ry', 'base[2]']], 'f3prox-f3tip':[[0.068, 0, 0.0325],['ry','distal[0]']]}
		#

		proximal_locations = np.array([	[0.017, 0, 0.0155], 
										[0.025, 0, 0.0155], 
										[0.033, 0, 0.0155], 
										[0.041, 0, 0.0155], 
										[0.049, 0, 0.0155]]).T
		
		distal_locations = np.array([	[0.01973, 0, 0.0155], 
										[0.02773, 0, 0.0155], 
										[0.03573, 0, 0.0155]]).T

		dlh = range(3) # distal tactile locations handframe
		plh = range(3) # proximal tactile locations handframe
		proximal_axes = range(3) # proximal tangent and normal axes
		distal_axes = range(3) # distal tangent and normal axes
		axes = range(3)
		knuckles = [np.pi/2, np.pi/2, -np.pi/2]#[np.pi/2 - motor_position[3], np.pi + motor_position[3], -np.pi/2]
		offset_distal = 3 * [np.array([[0.068, 0, 0.0325]]).T]
		offset_proximal = 3 * [np.array([[0.00825, 0, 0]]).T]
		offset_knuckle = [np.array([[0.026,   0.0354,  0.076]]).T, np.array([[-0.026,   0.0354,  0.076]]).T, np.array([[0.00,   -0.0354,  0.076]]).T]

		for i in range(3):
			plh[i] = Rz(-knuckles[i]) * ( Ry(-base_joints[i]) * proximal_locations + offset_proximal[i] * np.array([[1,1,1,1,1]]) ) + offset_knuckle[i] * np.array([[1,1,1,1,1]])
			dlh[i] = Rz(-knuckles[i]) * ( Ry(-base_joints[i]) * (Ry(-distal_joints[i]) * distal_locations + offset_distal[i] * np.array([1,1,1])) + offset_proximal[i] * np.array([1,1,1]) ) + offset_knuckle[i] * np.array([1,1,1])
			proximal_axes[i] = Rz(-knuckles[i]) * Ry(-base_joints[i]) * np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]]).T
			distal_axes[i] = Rz(-knuckles[i]) * Ry(-base_joints[i]) * Ry(-distal_joints[i]) * np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]]).T
			
		finger = ['f1', 'f2', 'f3']
		link = ['proximal', 'distal']
		names = []
		values = []
		for i in range(3):
			names += ['f%i_knuckle'%i]
			values += [Point(offset_knuckle[i][0], offset_knuckle[i][1], offset_knuckle[i][2])]
			for j in range(2):
				sensors = range(3) if link[j] == 'distal' else range(5)
				# pack tactile data
				for s in sensors:
					names += [finger[i] + '_' + link[j] + '_tactile_' + str(s+1)]
					values += [Point(dlh[i][0,s], dlh[i][1,s], dlh[i][2,s])] if link[j] == 'distal' else [Point(plh[i][0,s], plh[i][1,s], plh[i][2,s])]
				# pack normal & tangent data
				names += [finger[i] + '_' + link[j] + '_tangent']
				values += [Point(distal_axes[i][0,0], distal_axes[i][1,0], distal_axes[i][2,0])] if link[j] == 'distal' else [Point(distal_axes[i][0,0], distal_axes[i][1,0], distal_axes[i][2,0])]
				names += [finger[i] + '_' + link[j] + '_normal']
				values += [Point(distal_axes[i][0,1], distal_axes[i][1,1], distal_axes[i][2,1])] if link[j] == 'distal' else [Point(distal_axes[i][0,1], distal_axes[i][1,1], distal_axes[i][2,1])]
				names += [finger[i] + '_' + link[j] + '_jointaxis']
				values += [Point(distal_axes[i][0,2], distal_axes[i][1,2], distal_axes[i][2,2])] if link[j] == 'distal' else [Point(distal_axes[i][0,2], distal_axes[i][1,2], distal_axes[i][2,2])]
		return names, values

class SensorPublisher:
	def __init__(self):
		rospy.init_node('reflex_joints', anonymous=True)
		
		self.motor_state = MotorStateList()
		self.encoder_state = Encoders()
		self.encoder_state.proximal[0] = -1
		
		rospy.set_param('ENCODER_ZERO', ENCODER_ZERO)
		rospy.set_param('SPOOL_ZERO', SPOOL_ZERO[0:3])
		
		self.pub = rospy.Publisher('/reflex/joints', Joints)
		# self.fk_pub = rospy.Publisher('/reflex/kinematics', Kinematics)
	
		rospy.loginfo("loading servomap")
		servomap = rospy.get_param("servomap")
		self.servomap = [servomap[d] for d in servomap['motor_order']]
	
		slow_topic = '/motor_states/reflex_port'
		rospy.loginfo('waiting for topic %s'%slow_topic)	
		rospy.Subscriber(slow_topic, MotorStateList, self.slow_update)
		while len(self.motor_state.motor_states) == 0 and not rospy.is_shutdown():
			rospy.sleep(0.01)
	
		fast_topic = '/reflex/proximal_encoders'
		rospy.loginfo("waiting for topic %s", fast_topic)
		rospy.Subscriber(fast_topic, Encoders, self.fast_update)	
		while self.encoder_state.proximal[0] == -1 and not rospy.is_shutdown():
			rospy.sleep(0.01)
		
		rospy.loginfo('done -- running')
		rospy.spin()	
		
	def slow_update(self, motorlist_msg):
		self.motor_state = motorlist_msg

		
	def fast_update(self, encoders_msg):
		self.encoder_state = encoders_msg
		self.broadcast()

	def broadcast(self):
		spool_position = range(4)
		for i in range(4):
			if self.servomap[i]['invert']:
				spool_position[i] = self.servomap[i]['reference'] - self.motor_state.motor_states[i].position * 5.236 / 1023  - SPOOL_ZERO[i]
			else: 
				spool_position[i] = self.motor_state.motor_states[i].position * 5.236 / 1023 - self.servomap[i]['reference'] - SPOOL_ZERO[i]	


		spool_position[3] = self.servomap[3]['ratio'] * spool_position[3] # correct for transmission ratio in gearing between finger bases and motor		
		
		base_joints = range(3)
		base_joints = [self.encoder_state.proximal[i] - ENCODER_ZERO[i] for i in range(3)]

		distal_joints = range(3)
		error = range(3)
		for i in range(3):
			distal_joints[i] = max(0, 1.571 / 10 * (7 * spool_position[i] - 10 * base_joints[i]))
			error[i] = self.motor_state.motor_states[i].error * 5.236 / 1023

		self.pub.publish(spool_position[0:3], base_joints, distal_joints, error,spool_position[3])

		names, values = forward_kinematics(base_joints, distal_joints)
		# self.fk_pub.publish(names, values)


if __name__ == '__main__':
	try:
		lc = SensorPublisher()

	except rospy.ROSInterruptException:
		pass

