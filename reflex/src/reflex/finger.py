#############################################################################
# Copyright 2015 Right Hand Robotics
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

__author__ = 'Eric Schneider'
__copyright__ = 'Copyright (c) 2015 RightHand Robotics'
__license__ = 'Apache License 2.0'
__maintainer__ = 'RightHand Robotics'
__email__ = 'reflex-support@righthandrobotics.com'


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

    def _receive_state_cb(self, data):
        '''
        Stores the incoming finger data
        '''
        self.finger_msg = data
