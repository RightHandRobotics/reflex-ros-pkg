#!/usr/bin/env python

##########################################################
# Class to track state changes for debugging purposes
##########################################################

import rospy
from copy import deepcopy
from reflex_msgs.msg import StateDebug

class StateDebugger():
    def __init__(self):
        self.debug_pub = rospy.Publisher('/reflex/state_debug',
                                         StateDebug,
                                         queue_size=1)
        self.state = StateDebug()
        self.state_old = StateDebug()
        self.locked = False

    def track_fingers(self, control_mode):
        for idx in range(3):
            self.state.finger[idx].mode = control_mode[idx]

    def set_finger_state(self, idx, msg):
        self.state.finger[idx].state = msg

    def set_fingers_working(self, working):
        for idx in range(3):
            self.state.finger[idx].working = working[idx]

    def track_preshape(self):
        self.locked = True
        self.state = StateDebug()
        self.state.preshape.mode = 'goto'
        self.state.preshape.working = True
        self.state.preshape.state = 'Waiting for (arrived) or (blocked)'
        self.publish_state()

    def end_preshape_tracking(self, msg):
        self.state.preshape.working = False
        self.state.preshape.state = msg
        self.publish_state()
        self.locked = False

    def publish_state(self):
        self.debug_pub.publish(self.state)
        if not self.locked:
            self.state_old = deepcopy(self.state)

    def publish_state_if_changed(self):
        if not self.states_equal():
            self.publish_state()

    def states_equal(self):
        if self.state == None or self.state_old == None:
            return False
        for idx in range(3):
            if self.state.finger[idx].mode != self.state_old.finger[idx].mode \
              or self.state.finger[idx].working != self.state_old.finger[idx].working \
              or self.state.finger[idx].state != self.state_old.finger[idx].state:
                return False
        if self.state.preshape.mode != self.state_old.preshape.mode \
          or self.state.preshape.working != self.state_old.preshape.working \
          or self.state.preshape.state != self.state_old.preshape.state:
            return False
        return True
