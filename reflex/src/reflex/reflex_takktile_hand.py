#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty

import reflex_msgs.msg
import reflex_msgs.srv
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
                                             reflex_msgs.msg.RadianServoCommands, queue_size=10)
        rospy.Subscriber(self.namespace + '/command',
                         reflex_msgs.msg.ReflexCommand, self.receive_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_position',
                         reflex_msgs.msg.PoseCommand, self.receive_angle_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_velocity',
                         reflex_msgs.msg.VelocityCommand, self.receive_vel_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_motor_torque',
                         reflex_msgs.msg.TorqueCommand, self.receive_torque_cmd_cb)
        rospy.Subscriber(self.namespace + '/hand_state',
                         reflex_msgs.msg.Hand, self.receive_hand_state_cb)
        self.set_speed_service = rospy.ServiceProxy(self.namespace + '/set_speed', reflex_msgs.srv.SetSpeed)
        self.calibrate_fingers_service = rospy.ServiceProxy(self.namespace + '/calibrate_fingers', Empty)
        self.calibrate_tactile_service = rospy.ServiceProxy(self.namespace + '/calibrate_tactile', Empty)
        self.comms_timeout = 5.0  # Seconds with no communications until hand stops
        self.latest_update = rospy.get_rostime()
        rospy.loginfo('ReFlex Takktile hand has started, waiting for commands...')

    def receive_cmd_cb(self, data):
        self.disable_torque_control()
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)

    def receive_angle_cmd_cb(self, data):
        self.disable_torque_control()
        self.reset_speeds()
        self.set_angles(data)

    def receive_vel_cmd_cb(self, data):
        self.disable_torque_control()
        self.set_velocities(data)

    def receive_torque_cmd_cb(self, data):
        self.disable_torque_control()
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
        self.motors[self.namespace + '_f1'].set_motor_angle(pose.f1)
        self.motors[self.namespace + '_f2'].set_motor_angle(pose.f2)
        self.motors[self.namespace + '_f3'].set_motor_angle(pose.f3)
        self.motors[self.namespace + '_preshape'].set_motor_angle(pose.preshape)

    def set_velocities(self, velocity):
        self.motors[self.namespace + '_f1'].set_motor_velocity(velocity.f1)
        self.motors[self.namespace + '_f2'].set_motor_velocity(velocity.f2)
        self.motors[self.namespace + '_f3'].set_motor_velocity(velocity.f3)
        self.motors[self.namespace + '_preshape'].set_motor_velocity(velocity.preshape)

    def set_speeds(self, speed):
        self.motors[self.namespace + '_f1'].set_motor_speed(speed.f1)
        self.motors[self.namespace + '_f2'].set_motor_speed(speed.f2)
        self.motors[self.namespace + '_f3'].set_motor_speed(speed.f3)
        self.motors[self.namespace + '_preshape'].set_motor_speed(speed.preshape)

    def set_torque_cmds(self, torque):
        self.motors[self.namespace + '_f1'].set_torque_cmd(torque.f1)
        self.motors[self.namespace + '_f2'].set_torque_cmd(torque.f2)
        self.motors[self.namespace + '_f3'].set_torque_cmd(torque.f3)
        self.motors[self.namespace + '_preshape'].set_torque_cmd(torque.preshape)

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def disable_torque_control(self):
        for ID, motor in self.motors.items():
            motor.disable_torque_control()
        rospy.sleep(0.05)  # Lets commands stop before allowing any other actions

    def enable_torque_control(self):
        rospy.sleep(0.05)  # Lets other actions happen before beginning constant torque commands
        for ID, motor in self.motors.items():
            motor.enable_torque_control()

    def calibrate_fingers(self):
        self.calibrate_fingers_service()

    def calibrate_tactile(self):
        self.calibrate_tactile_service()

    def publish_motor_commands(self):
        '''
        Checks whether motors have updated their states and, if so, publishes
        their commands
        '''
        speed_update_occurred = False
        for ID, motor in self.motors.items():
            speed_update_occurred |= motor.speed_update_occurred
        position_update_occurred = False
        for ID, motor in self.motors.items():
            position_update_occurred |= motor.position_update_occurred
        if speed_update_occurred:
            self.publish_speed_commands()
        if position_update_occurred:
            self.publish_position_commands()

    def publish_speed_commands(self):
        '''
        Queries the motors for their speed and position setpoints and publishes
        those to the appropriate topics for reflex_driver
        '''
        motor_speed_cmd = reflex_msgs.srv.SetSpeedRequest(
            [self.motors['/reflex_takktile_f1'].get_commanded_speed(),
             self.motors['/reflex_takktile_f2'].get_commanded_speed(),
             self.motors['/reflex_takktile_f3'].get_commanded_speed(),
             self.motors['/reflex_takktile_preshape'].get_commanded_speed()])
        self.set_speed_service(motor_speed_cmd)
        for ID, motor in self.motors.items():
            motor.speed_update_occurred = False
        rospy.sleep(0.02)  # Without a sleep the hand freezes up

    def publish_position_commands(self):
        '''
        Queries the motors for their speed and position setpoints and publishes
        those to the appropriate topics for reflex_driver
        '''
        motor_pos_cmd = reflex_msgs.msg.RadianServoCommands(
            [self.motors['/reflex_takktile_f1'].get_commanded_position(),
             self.motors['/reflex_takktile_f2'].get_commanded_position(),
             self.motors['/reflex_takktile_f3'].get_commanded_position(),
             self.motors['/reflex_takktile_preshape'].get_commanded_position()])
        self.motor_cmd_pub.publish(motor_pos_cmd)
        for ID, motor in self.motors.items():
            motor.position_update_occurred = False
        rospy.sleep(0.01)


def main():
    rospy.sleep(2.0)  # To allow services and parameters to load
    hand = ReflexTakktileHand()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        hand.publish_motor_commands()
        if (rospy.get_rostime().secs > (hand.latest_update.secs + hand.comms_timeout)):
            rospy.logfatal('Hand going down, no ethernet data for %d seconds', hand.comms_timeout)
            rospy.signal_shutdown('Comms timeout')
        r.sleep()

if __name__ == '__main__':
    main()
