import numpy as np

import reflex_msgs.msg


class Finger(object):
    def __init__(self):
        self.finger_msg = reflex_msgs.msg.Finger()

    def get_proximal_angle(self):
        return self.finger_msg.proximal

    def get_distal_angle_approximation(self):
        return self.finger_msg.distal_approx

    def is_finger_in_contact(self):
        return sum(self.finger_msg.contact) > 0

    def is_proximal_in_contact(self):
        return sum(self.finger_msg.contact[:5]) > 0

    def is_distal_in_contact(self):
        return sum(self.finger_msg.contact[5:]) > 0

    def receive_state_cb(self, data):
        '''
        Stores the incoming finger data
        '''
        self.finger_msg = data
