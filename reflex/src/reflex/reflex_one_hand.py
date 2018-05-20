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

__author__ = 'Eric Schneider'
__copyright__ = 'Copyright (c) 2015 RightHand Robotics'
__license__ = 'Apache License 2.0'
__maintainer__ = 'RightHand Robotics'
__email__ = 'reflex-support@righthandrobotics.com'


from os.path import join
import yaml
import time

from dynamixel_msgs.msg import JointState
from dynamixel_msgs.msg import Encoder
import rospkg
import rospy
from std_srvs.srv import Empty

from reflex_hand import ReflexHand
from reflex_usb_motor import ReflexUSBMotor
import reflex_msgs.msg

motor_names = ['_f1', '_f2', '_f3', '_preshape1', '_preshape2']

class ReflexOneHand(ReflexHand):
    def __init__(self):
        self.usb_hand_type = rospy.get_param('usb_hand_type')
        self.init_namespace = '/' + self.usb_hand_type
        super(ReflexOneHand, self).__init__(self.init_namespace, ReflexUSBMotor)
        self.hand_state_pub = rospy.Publisher(self.namespace + '/hand_state',
                                              reflex_msgs.msg.Hand, queue_size=10)
        self.encoder_last_value = [0, 0, 0]  #This will be updated constantly in _receive_enc_state_cb()
        self.encoder_offset = [0, 0, 0]
        self.enc_scale = (2 * 3.141596) / 16383
        self.proximal_angle = [0,0,0]
        self.distal_approx = [0,0,0]
        self.calibration_error = 15
        self.num_calibration_trials = 20
        rospy.Service(self.namespace + '/calibrate_manual', Empty, self.calibrate_manual)
        if (self.usb_hand_type == "reflex_plus"):
            self.enc_subscriber = rospy.Subscriber('/encoder_states', Encoder, self._receive_enc_state_cb)
            rospy.Service(self.namespace + '/calibrate_fingers', Empty, self.calibrate_auto)
            self.encoder_zero_point = rospy.get_param('/enc_zero_points')
        else:
            rospy.Service(self.namespace + '/calibrate_fingers', Empty, self.calibrate_manual)

    def set_angles(self, pose):
        self.motors[self.namespace + '_f1'].set_motor_angle(pose.f1)
        self.motors[self.namespace + '_f2'].set_motor_angle(pose.f2)
        self.motors[self.namespace + '_f3'].set_motor_angle(pose.f3)
        self.motors[self.namespace + '_preshape1'].set_motor_angle(pose.preshape1)
        self.motors[self.namespace + '_preshape2'].set_motor_angle(pose.preshape2)

    def set_velocities(self, velocity):
        self.motors[self.namespace + '_f1'].set_motor_velocity(velocity.f1)
        self.motors[self.namespace + '_f2'].set_motor_velocity(velocity.f2)
        self.motors[self.namespace + '_f3'].set_motor_velocity(velocity.f3)
        self.motors[self.namespace + '_preshape1'].set_motor_velocity(velocity.preshape1)
        self.motors[self.namespace + '_preshape2'].set_motor_velocity(velocity.preshape2)

    def set_speeds(self, speed):
        self.motors[self.namespace + '_f1'].set_motor_speed(speed.f1)
        self.motors[self.namespace + '_f2'].set_motor_speed(speed.f2)
        self.motors[self.namespace + '_f3'].set_motor_speed(speed.f3)
        self.motors[self.namespace + '_preshape1'].set_motor_speed(speed.preshape1)
        self.motors[self.namespace + '_preshape2'].set_motor_speed(speed.preshape2)

    def set_force_cmds(self, torque):
        self.motors[self.namespace + '_f1'].set_force_cmd(torque.f1)
        self.motors[self.namespace + '_f2'].set_force_cmd(torque.f2)
        self.motors[self.namespace + '_f3'].set_force_cmd(torque.f3)
        self.motors[self.namespace + '_preshape1'].set_force_cmd(torque.preshape1)
        self.motors[self.namespace + '_preshape2'].set_force_cmd(torque.preshape2)

    def _receive_enc_state_cb(self, data):
        #Receives and processes the encoder state
        #print("encoder 1: " + str(data.encoders[0]) + " encoder 2: " + str(data.encoders[1]) + " encoder 3: " + str(data.encoders[2]))
        self.update_encoder_offset(data.encoders)
        self.encoder_last_value = data.encoders[:]
        motor_angles = [0, 0, 0]
        for i in range(3):
            self.proximal_angle[i] = self.calc_proximal_angle(data.encoders[i], self.encoder_zero_point[i], self.encoder_offset[i])
            raw_motor_angle = self.motors[self.namespace + motor_names[i]].get_current_raw_motor_angle()
            motor_joint_angle = self.motors[self.namespace + motor_names[i]].get_current_joint_angle() #self.calc_motor_angle(self.MOTOR_TO_JOINT_INVERTED[i], raw_motor_angle, self.MOTOR_TO_JOINT_GEAR_RATIO[i], self.motor_zero_point[i])
            #motor_angles[i] = raw_motor_angle
            self.distal_approx[i] = self.calc_distal_angle(motor_joint_angle, self.proximal_angle[i])
        #print motor_angles

    def _receive_cmd_cb(self, data):
        self.disable_force_control()
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)

    def _receive_angle_cmd_cb(self, data):
        self.disable_force_control()
        self.reset_speeds()
        self.set_angles(data)

    def _receive_vel_cmd_cb(self, data):
        self.disable_force_control()
        self.set_velocities(data)

    def _receive_force_cmd_cb(self, data):
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
        state = reflex_msgs.msg.Hand()
        # motor_names = ('_f1', '_f2', '_preshape1')
        motor_names = ('_f1', '_f2', '_f3', '_preshape1', '_preshape2')
        for i in range(len(motor_names)):
            state.motor[i] = self.motors[self.namespace + motor_names[i]].get_motor_msg()
        for i in range(3):
            state.finger[i].proximal = self.proximal_angle[i]
            state.finger[i].distal_approx = self.distal_approx[i]

        self.hand_state_pub.publish(state)

    def calibrate_manual(self, data=None):
        #Manual hand calibration
        for motor in sorted(self.motors):
            rospy.loginfo("Calibrating motor " + motor)
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
motor, or 'q' to indicate that the zero point has been reached\n")
            while not command.lower() == 'q':
                if command.lower() == 't' or command.lower() == 'tt':
                    print "Tightening motor " + motor
                    self.motors[motor].tighten(0.35 * len(command) - 0.3)
                elif command.lower() == 'l' or command.lower() == 'll':
                    print "Loosening motor " + motor
                    self.motors[motor].loosen(0.35 * len(command) - 0.3)
                else:
                    print "Didn't recognize that command, use 't', 'l', or 'q'"
                command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
            rospy.loginfo("Saving current position for %s as the zero point", motor)
            self.motors[motor]._set_local_motor_zero_point()
        print "Calibration complete, writing data to file"
        self._zero_current_pose()
        return []

    def calibrate_auto(self, data=None):
        #Auto calibrates the hand using encoder data and moving until motion is detected
        for i in range(len(motor_names)):
            time.sleep(0.125)
            j=0
            while(1):
                enc_pos = self.encoder_last_value[i]
                if (j==0):
                    j=1
                    last = enc_pos
                if ((abs(enc_pos-last) < self.calibration_error)):
                    self.motors[self.namespace + motor_names[i]].tighten()
                    time.sleep(0.2)
                    last = enc_pos
                else:
                    break
            self.motors[self.namespace + motor_names[i]]._set_local_motor_zero_point()
        self.motors[self.namespace + motor_names[3]]._set_local_motor_zero_point()  #Zero preshape in place
        print "Calibration complete, writing data to file"
        self._zero_current_pose()
        self.calibrate_encoders_locally(self.encoder_last_value)
        for i in range(len(motor_names)):
            self.motors[self.namespace + motor_names[i]].set_motor_angle(goal_pos = 0.5)
        for i in range(len(motor_names)):
            self.motors[self.namespace + motor_names[i]].set_motor_angle(goal_pos = 0.0)
        return []

    def _write_zero_point_data_to_file(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_usb_path = rospack.get_path("reflex")
        yaml_path = "yaml"
        file_path = join(reflex_usb_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def _zero_current_pose(self):
        data = dict(
            reflex_one_f1=dict(zero_point=self.motors[self.namespace + '_f1'].get_current_raw_motor_angle()),
            reflex_one_f2=dict(zero_point=self.motors[self.namespace + '_f2'].get_current_raw_motor_angle()),
            reflex_one_f3=dict(zero_point=self.motors[self.namespace + '_f3'].get_current_raw_motor_angle()),
            reflex_one_preshape1=dict(zero_point=self.motors[self.namespace + '_preshape1'].get_current_raw_motor_angle()),
            reflex_one_preshape2=dict(zero_point=self.motors[self.namespace + '_preshape2'].get_current_raw_motor_angle())
        )
        self._write_zero_point_data_to_file(self.usb_hand_type + '_motor_zero_points.yaml', data)

    #Encoder data processing functions are based off encoder functions used
    #For the reflex_takktile hand as in reflex_driver_node.cpp
    def calibrate_encoders_locally(self, data):
        #Capture the current encoder position locally as zero and save to file
        for i in range(0, 3):
            self.encoder_zero_point[i] = data[i]*self.enc_scale
            self.encoder_offset[i] = 0;
        data = dict(enc_zero_points = self.encoder_zero_point)
        self._write_zero_point_data_to_file('reflex_one_zero_points.yaml', data)

    def update_encoder_offset(self, raw_value):
        #Given a raw and past (self.encoder_last_value) value, track encoder wrapes (self.enc_offset)
        offset = self.encoder_offset[:]

        for i in range(0,3):
            if (offset[i]==-1):
                #This happens at start up
                offset[i] = 0
            else:
                #If the encoder jumps, that means it wrapped a revolution
                if (self.encoder_last_value[i] - raw_value[i] > 5000):
                    offset[i] = offset[i] + 16383
                elif (self.encoder_last_value[i] - raw_value[i] < -5000):
                    offset[i] = offset[i] - 16383

        self.encoder_offset = offset[:]

    def calc_proximal_angle(self, raw_value, zero, offset):
        #Calculates actual proximal angle using raw sensor data, and
        #the encoder "zero" point for that encoder
        wrapped_enc_value = raw_value + offset
        rad_value = wrapped_enc_value*self.enc_scale
        return (zero - rad_value)

    def calc_distal_angle(self, joint_angle, proximal):
        #Calculates the distal angle, "tendon spooled out" - "proximal encoder" angles
        #Could be improved
        diff = joint_angle - proximal
        if (diff<0):
            return 0
        else:
            return diff

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
