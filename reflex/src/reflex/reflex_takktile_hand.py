#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from reflex_msgs.msg import ReflexCommand
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import TorqueCommand
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
        self.latest_update = rospy.get_rostime()
        self.comms_timeout = 5.0  # Seconds with no communications until hand stops
        rospy.Subscriber(self.namespace + '/command', ReflexCommand, self.receive_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_position', PoseCommand, self.receive_angle_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_velocity', VelocityCommand, self.receive_vel_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_motor_torque', TorqueCommand, self.receive_torque_cmd_cb)
        rospy.Subscriber(self.namespace + '/hand_state', Hand, self.receive_hand_state_cb)
        rospy.loginfo('ReFlex Takktile hand has started, waiting for commands...')

    def receive_cmd_cb(self, data):
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)
        self.publish_motor_commands()
        self.disable_torque_control()

    def receive_angle_cmd_cb(self, data):
        self.reset_speeds()
        self.set_angles(data)
        self.publish_motor_commands()
        self.disable_torque_control()

    def receive_vel_cmd_cb(self, data):
        self.set_velocities(data)
        self.publish_motor_commands()
        self.disable_torque_control()

    def receive_torque_cmd_cb(self, data):
        self.reset_speeds()
        self.set_torque_cmds(data)
        self.enable_torque_control()

    def receive_hand_state_cb(self, data):
        self.latest_update = rospy.get_rostime()
        self.motors['/reflex_takktile_f1'].receive_state_cb(data.motor[0])
        self.motors['/reflex_takktile_f2'].receive_state_cb(data.motor[1])
        self.motors['/reflex_takktile_f3'].receive_state_cb(data.motor[2])
        self.motors['/reflex_takktile_preshape'].receive_state_cb(data.motor[3])
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

    def set_torque_cmds(self, torque):
        self.motors[self.namespace + '_f1'].set_torque_cmd(torque.f1)
        self.motors[self.namespace + '_f2'].set_torque_cmd(torque.f2)
        self.motors[self.namespace + '_f3'].set_torque_cmd(torque.f3)
        self.motors[self.namespace + '_preshape'].set_torque_cmd(torque.preshape)

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def calibrate_fingers(self):
        self.calibrate_fingers_service()

    def calibrate_tactile(self):
        self.calibrate_tactile_service()

    def disable_torque_control(self):
        for ID, motor in self.motors.items():
            motor.disable_torque_control()

    def enable_torque_control(self):
        for ID, motor in self.motors.items():
            motor.enable_torque_control()

    def publish_motor_commands_on_state_update(self):
        '''
        Checks whether motors have updated their states and, if so, publishes
        their commands
        '''
        update_occurred = False
        for ID, motor in self.motors.items():
            if motor.update_occurred:
                update_occurred = True
        if update_occurred:
            self.publish_motor_commands()

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
        rospy.sleep(0.02)  # Without a sleep the hand freezes up
        self.set_speed_service(motor_speed_cmd)
        rospy.sleep(0.02)
        self.motor_cmd_pub.publish(motor_pos_cmd)
        for ID, motor in self.motors.items():
            motor.update_occurred = False


def main():
    rospy.sleep(2.0)  # To allow services and parameters to load
    hand = ReflexTakktileHand()
    while not rospy.is_shutdown():
        hand.publish_motor_commands_on_state_update()
        now = rospy.get_rostime()
        if (now.secs > (hand.latest_update.secs + hand.comms_timeout)):
            rospy.logfatal('reflex_takktile_hand going down, no hand data for %d seconds', hand.comms_timeout)
            rospy.signal_shutdown('Comms timeout')
        rospy.sleep(0.05)

if __name__ == '__main__':
    main()
