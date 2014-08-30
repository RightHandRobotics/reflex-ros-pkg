#!/usr/bin/env python

####################################################
#
# Parse info from TakkTile sensors into higher-level
#	information
#
####################################################


# import roslib; roslib.load_manifest('reflex_smarts')
import rospy

import numpy as np

from reflex_msgs.msg import Sensors
from takktile_ros.msg import Contact, Info, Touch

# some parameters, all in meters
SENSOR_PLACEMENT = [0.017,  0.025,  0.033,  0.041,  0.049, 0.0875, 0.0955, 0.1035, 0.112]
LINK_END = [0.050, 0.110]
PALM_THRESHOLD = 100

class SensorPublisher:
	def __init__(self, topic_in='/takktile', topic_out='/reflex/contact'):
		rospy.init_node('reflex_tactile', anonymous=True)
		self.info = False

		self.output = {}

		self.pub = rospy.Publisher(topic_out, Sensors)
		
		info = rospy.Subscriber(topic_in + '/sensor_info', Info, self.load_info)
		rospy.loginfo('waiting for topic %s/sensor_info'%topic_in)
		while not self.info and not rospy.is_shutdown():
			rospy.sleep(0.1)
	
		rospy.loginfo('sensor info recorded')
		rospy.loginfo('waiting for topic %s/contact'%topic_in)	
		rospy.Subscriber(topic_in + '/contact', Contact, self.update_contact)
		rospy.loginfo('waiting for topic %s/calibrated'%topic_in)
		rospy.Subscriber(topic_in + '/calibrated', Touch, self.update_pressure)	

		#self.contact_link_parser = ContactLinkParser()
		#self.contact_location_parser = ContactLocationParser()
	
		rospy.loginfo('done -- running')
		rospy.spin()	

	def load_info(self, info):
		if self.info:
			return
		
		#mapping = [['f2_proximal',[20,21,22,23,24]], ['f2_distal',[25,26,27]],
		#           ['f1_proximal',[ 0, 1, 2, 3, 4]], ['f1_distal',[ 5, 6, 7]],
		#	   ['f3_proximal',[30,31,32,33,34]], ['f3_distal',[35,36,37]],
		#	]
			  #['palm_pad1_distal', [15]], ['palm_pad1_proximal',[16]], 

		mapping = [['f3_proximal',[20,21,22,23,24]], ['f3_distal',[25,26,27]],
		           ['f2_proximal',[ 0, 1, 2, 3, 4]], ['f2_distal',[ 5, 6, 7]],
			   ['f1_proximal',[10,11,12,13,14]], ['f1_distal',[15,16,17]],
			]
			  #['palm_pad1_distal', [15]], ['palm_pad1_proximal',[16]], 
			  #['palm_pad2_distal',[19]], ['palm_pad2_proximal', [18]],
			  #['palm_pad3', [17]]]
		
		pads, es = zip(*mapping)
		expected_sensors = []
		for e in es:
			expected_sensors += e
		expected_sensors.sort()

		print expected_sensors

		if not all((s in info.indexes for s in expected_sensors)):
			missing = []
			for i in expected_sensors:
				if not i in info.indexes:
					missing += [i]
			print 'missing sensors: ', missing
			print 'exiting'
			exit()

		self.indices = {}	
		for [pad, indices] in mapping:
			self.indices[pad] = []
			for j in indices:
				self.indices[pad] += [expected_sensors.index(j)]

		self.info = True

	def update_contact(self, contact):
		c = contact.pressure
		for j in range(3):
			for pad in ['proximal', 'distal']:
				self.output['f%i_%s'%(j+1,pad)] = any((c[i] for i in self.indices['f%i_%s'%(j+1,pad)]))
				
		self.broadcast()

	def update_pressure(self, calibrated):
		c = list(calibrated.pressure)
		for pad in filter(lambda key: 'palm' in key, self.indices.keys()):
			self.output[pad] = any([abs(c[i]) > PALM_THRESHOLD for i in self.indices[pad]])

	def broadcast(self):
		keys = self.output.keys()
		self.pub.publish(keys, [self.output[key] for key in keys])

if __name__ == '__main__':
	try:
		lc = SensorPublisher()

	except rospy.ROSInterruptException:
		pass