import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty

import reflex_msgs.msg
import motor


class ReflexHand():
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the hand with a preceding
        slash, e.g. /reflex_takktile or /reflex_sf
        '''
        self.namespace = name
        rospy.init_node('reflex_hand')
        rospy.loginfo('Starting up the hand')
        self.motors = {self.namespace + '_f1': motor.Motor(self.namespace + '_f1'),
                       self.namespace + '_f2': motor.Motor(self.namespace + '_f2'),
                       self.namespace + '_f3': motor.Motor(self.namespace + '_f3'),
                       self.namespace + '_preshape': motor.Motor(self.namespace + '_preshape')}
        rospy.Subscriber(self.namespace + '/command',
                         reflex_msgs.msg.ReflexCommand, self.receive_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_position',
                         reflex_msgs.msg.PoseCommand, self.receive_angle_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_velocity',
                         reflex_msgs.msg.VelocityCommand, self.receive_vel_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_motor_force',
                         reflex_msgs.msg.ForceCommand, self.receive_force_cmd_cb)
        rospy.loginfo('ReFlex hand has started, waiting for commands...')

    def receive_cmd_cb(self, data):
        raise NotImplementedError

    def receive_angle_cmd_cb(self, data):
        raise NotImplementedError

    def receive_vel_cmd_cb(self, data):
        raise NotImplementedError)

    def receive_force_cmd_cb(self, data):
        raise NotImplementedError

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

    def set_force_cmds(self, torque):
        self.motors[self.namespace + '_f1'].set_force_cmd(torque.f1)
        self.motors[self.namespace + '_f2'].set_force_cmd(torque.f2)
        self.motors[self.namespace + '_f3'].set_force_cmd(torque.f3)
        self.motors[self.namespace + '_preshape'].set_force_cmd(torque.preshape)

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def disable_force_control(self):
        for ID, motor in self.motors.items():
            motor.disable_force_control()
        rospy.sleep(0.05)  # Lets commands stop before allowing any other actions

    def enable_force_control(self):
        rospy.sleep(0.05)  # Lets other actions happen before beginning constant torque commands
        for ID, motor in self.motors.items():
            motor.enable_force_control()
