#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from reflex_msgs.msg import ReflexCommand
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import RadianServoCommands
from reflex_msgs.msg import Hand
from reflex_msgs.srv import SetSpeed, SetSpeedRequest
import motor


class ReflexTakktileHand():
    def __init__(self):
        self.namespace = '/reflex_takktile'
        rospy.init_node('reflex_takktile')
        rospy.loginfo('Starting up the ReFlex Takktile hand')
        self.motors = {self.namespace + '_f1': motor.Motor(self.namespace + '_f1'),
                       self.namespace + '_f2': motor.Motor(self.namespace + '_f2'),
                       self.namespace + '_f3': motor.Motor(self.namespace + '_f3'),
                       self.namespace + '_preshape': motor.Motor(self.namespace + '_preshape')}
        self.motor_cmd_pub = rospy.Publisher(self.namespace + '/radian_hand_command',
                                             RadianServoCommands, queue_size=10)
        self.set_speed_service = rospy.ServiceProxy(self.namespace + '/set_speed', SetSpeed)
        self.calibrate_fingers_service = rospy.ServiceProxy(self.namespace + '/calibrate_fingers', Empty)
        self.calibrate_tactile_service = rospy.ServiceProxy(self.namespace + '/calibrate_tactile', Empty)
        rospy.Subscriber(self.namespace + '/command', ReflexCommand, self.receive_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_position', PoseCommand, self.receive_angle_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_velocity', VelocityCommand, self.receive_vel_cmd_cb)
        rospy.Subscriber(self.namespace + '/hand_state', Hand, self.receive_hand_state_cb)
        rospy.loginfo('ReFlex Takktile hand has started, waiting for commands...')

    def receive_cmd_cb(self, data):
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)
        self.publish_motor_commands()

    def receive_angle_cmd_cb(self, data):
        self.reset_speeds()
        self.set_angles(data)
        self.publish_motor_commands()

    def receive_vel_cmd_cb(self, data):
        self.set_velocities(data)
        self.publish_motor_commands()

    def receive_hand_state_cb(self, data):
        # Todo(Eric): Do something about tactile data here
        pass

    def set_angles(self, pose):
        self.motors['/reflex_takktile_f1'].set_motor_angle(pose.f1)
        self.motors['/reflex_takktile_f2'].set_motor_angle(pose.f2)
        self.motors['/reflex_takktile_f3'].set_motor_angle(pose.f3)
        self.motors['/reflex_takktile_preshape'].set_motor_angle(pose.preshape)

    def set_velocities(self, velocity):
        self.motors['/reflex_takktile_f1'].set_motor_velocity(velocity.f1)
        self.motors['/reflex_takktile_f2'].set_motor_velocity(velocity.f2)
        self.motors['/reflex_takktile_f3'].set_motor_velocity(velocity.f3)
        self.motors['/reflex_takktile_preshape'].set_motor_velocity(velocity.preshape)

    def set_speeds(self, speed):
        self.motors['/reflex_takktile_f1'].set_motor_speed(speed.f1)
        self.motors['/reflex_takktile_f2'].set_motor_speed(speed.f2)
        self.motors['/reflex_takktile_f3'].set_motor_speed(speed.f3)
        self.motors['/reflex_takktile_preshape'].set_motor_speed(speed.preshape)

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def calibrate_fingers(self):
        self.calibrate_fingers_service()

    def calibrate_tactile(self):
        self.calibrate_tactile_service()

    def publish_motor_commands(self):
        '''
        Queries the motors for their speed and position setpoints and publishes
        those to the appropriate topics for reflex_driver
        '''
        motor_speed_cmd = SetSpeedRequest(
            [self.motors['/reflex_takktile_f1'].get_commanded_speed(),
             self.motors['/reflex_takktile_f2'].get_commanded_speed(),
             self.motors['/reflex_takktile_f3'].get_commanded_speed(),
             self.motors['/reflex_takktile_preshape'].get_commanded_speed()])
        motor_pos_cmd = RadianServoCommands(
            [self.motors['/reflex_takktile_f1'].get_commanded_position(),
             self.motors['/reflex_takktile_f2'].get_commanded_position(),
             self.motors['/reflex_takktile_f3'].get_commanded_position(),
             self.motors['/reflex_takktile_preshape'].get_commanded_position()])
        self.set_speed_service(motor_speed_cmd)
        rospy.sleep(0.03)  # Without a sleep the hand freezes up
        self.motor_cmd_pub.publish(motor_pos_cmd)


def main():
    hand = ReflexTakktileHand()
    rospy.spin()


if __name__ == '__main__':
    main()
