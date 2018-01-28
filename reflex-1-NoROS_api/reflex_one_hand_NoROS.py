#!/usr/bin/env python

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

__author__ = 'Michael Kaca, Harrison Young'
__copyright__ = 'Copyright (c) 2018 RightHand Robotics'
__license__ = 'Apache License 2.0'
__maintainer__ = 'RightHand Robotics'
__email__ = 'reflex-support@righthandrobotics.com'

import sys
from os.path import join, exists
import pickle
from reflex_hand_5_motors_ref1_NoROS import ReflexHand
from reflex_one_motor_NoROS import ReflexOneMotor
import threading
import time


class ReflexOneHand(ReflexHand):
    def __init__(self):

        super(ReflexOneHand, self).__init__('reflex_one', ReflexOneMotor)

        #self._set_zero_points_from_file('reflex_one_zero_points.yaml')
        motor_model = {}

        for motor in sorted(self.motors):
            motor_model[self.motors[motor].ID]= {}
            motor_model[self.motors[motor].ID]['Protocol'] = self.motors[motor].PROTOCOL
            motor_model[self.motors[motor].ID]['Model number'] = self.motors[motor].MODEL
        self.dxl_io.import_motors_model(motor_model)
        self.reset_speeds()
        self.hand_state = {}
        self.interval = 0.05
        background = threading.Thread(target=self.feedback, args=())
        background.daemon = True
        background.start()


    def feedback(self):
        start_time = time.clock()
        count = 0
        while(True):

            time.sleep(self.interval)
            for motor in sorted(self.motors):

                self.motors[motor].get_motor_feedback(self.dxl_io)
                self._publish_hand_state()






    def _receive_cmd_cb(self, data, print_data = False):
        if print_data:
            print data
        self.disable_force_control()
        self.set_speeds(data["velocity"])
        self.set_angles(data["pose"])

    def _receive_angle_cmd_cb(self, data, print_data = False):
        if print_data:
            print data
        self.disable_force_control()
        self.reset_speeds()
        self.set_angles(data)

    def _receive_vel_cmd_cb(self, data, print_data = False):
        if print_data:
            print data
        self.disable_force_control()
        self.set_velocities(data)

    def _receive_force_cmd_cb(self, data, print_data = False):
        if print_data:
            print data
        self.disable_force_control()
        self.reset_speeds()
        self.set_force_cmds(data)
        self.enable_force_control()

    def disable_torque(self):
        for ID, motor in self.motors.items():
            motor.disable_torque()

    def enable_torque(self):
        for ID, motor in self.motors.items():
            motor.enable_torque()

    def _publish_hand_state(self):

        motor_names = ('_f1', '_f2', '_f3', '_preshape1','_preshape2')
        for i in range(len(motor_names)):
            self.hand_state[motor_names[i]] = {}
            self.hand_state[motor_names[i]] = self.motors[self.namespace + motor_names[i]]._receive_state_cb(self.dxl_io)



    def calibrate(self, data=None):
        for motor in sorted(self.motors):
            print ("Calibrating motor " + self.motors[motor].name)
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
motor, or 'q' to indicate that the zero point has been reached\n")

            while not command.lower() == 'q':
                print "motor at ", self.motors[motor].position
                if command.lower() == 't' or command.lower() == 'tt':
                    print "Tightening motor " + self.motors[motor].name
                    self.motors[motor].tighten(self.dxl_io, 0.35 * len(command)- 0.3)
                elif command.lower() == 'l' or command.lower() == 'll':
                    print "Loosening motor " + self.motors[motor].name
                    self.motors[motor].loosen(self.dxl_io, 0.35 * len(command) - 0.3)
                else:
                    print "Didn't recognize that command, use 't', 'l', or 'q'"
                command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
            print("Saving current position for %s as the zero point", motor)
            self.motors[motor]._set_local_motor_zero_point()
        print "Calibration complete, writing data to file"
        self._zero_current_pose()
        return []

    def _write_zero_point_data_to_file(self, filename, data):
        print data
        with open(filename, 'wb') as handle:
            pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def _set_zero_points_from_file(self,filename):
        with open(filename, 'rb') as handle:
            zero_points = pickle.load(handle)
            for motor in sorted(self.motors):
                self.motors[motor].zero_point = zero_points[self.motors[motor].name]['zero_point']

    def _zero_current_pose(self):
        data = dict(
            reflex_one_f1=dict(zero_point=self.motors[self.namespace + '_f1'].position),
            reflex_one_f2=dict(zero_point=self.motors[self.namespace + '_f2'].position),
            reflex_one_f3=dict(zero_point=self.motors[self.namespace + '_f3'].position),
            reflex_one_preshape1=dict(zero_point=self.motors[self.namespace + '_preshape1'].position),
            reflex_one_preshape2=dict(zero_point=self.motors[self.namespace + '_preshape2'].position),
        )
        self._write_zero_point_data_to_file('reflex_one_zero_points.yaml', data)


def main():
    rospy.sleep(4.0)  # To allow services and parameters to load
    hand = ReflexOneHand()

    rospy.on_shutdown(hand.disable_torque)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        hand._publish_hand_state()
        r.sleep()


if __name__ == '__main__':
    main()
