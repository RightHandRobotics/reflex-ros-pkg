import rospy
from std_msgs.msg import Float64

import reflex_msgs.msg


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_takktile_f1
        '''
        self.name = name[1:]
        self.DEFAULT_MOTOR_SPEED = rospy.get_param(name + '/default_motor_speed')
        self.MAX_MOTOR_SPEED = rospy.get_param(name + '/max_motor_speed')
        self.MAX_MOTOR_TRAVEL = rospy.get_param(name + '/max_motor_travel')
        self.OVERLOAD_THRESHOLD = rospy.get_param(name + '/overload_threshold')
        self.motor_msg = reflex_msgs.msg.Motor()
        self.motor_cmd = 0.0
        self.speed = 0.0
        self.in_control_torque_mode = False
        self.torque_cmd = 0.0
        self.previous_load_control_output = 0.0
        self.previous_load_control_error = 0.0
        self.position_update_occurred = False
        self.speed_update_occurred = False
        self.reset_motor_speed()

    def get_current_joint_angle(self):
        return self.motor_msg.joint_angle

    def get_load(self):
        return self.motor_msg.load

    def get_motor_msg(self):
        return self.motor_msg

    def get_commanded_position(self):
        return self.motor_cmd

    def get_commanded_speed(self):
        return self.speed

    def set_motor_angle(self, goal_pos):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.motor_cmd = self.check_motor_angle_command(goal_pos)
        self.position_update_occurred = True

    def check_motor_angle_command(self, angle_command):
        '''
        Returns given command if within the allowable range,
        returns bounded command if out of range
        '''
        bounded_command = min(max(angle_command, 0.0), self.MAX_MOTOR_TRAVEL)
        return bounded_command

    def set_motor_speed(self, goal_speed):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.speed = self.check_motor_speed_command(goal_speed)
        self.speed_update_occurred = True

    def check_motor_speed_command(self, goal_speed):
        '''
        Returns absolute of given command if within the allowable range,
        returns bounded command if out of range. Always returns positive
        '''
        bounded_command = min(abs(goal_speed), self.MAX_MOTOR_SPEED)
        return bounded_command

    def reset_motor_speed(self):
        '''
        Resets speed to default
        '''
        self.speed = self.DEFAULT_MOTOR_SPEED
        self.speed_update_occurred = True

    def set_motor_velocity(self, goal_vel):
        '''
        Sets speed and commands finger in or out based on sign of velocity
        '''
        self.set_motor_speed(goal_vel)
        if goal_vel > 0.0:
            self.set_motor_angle(self.MAX_MOTOR_TRAVEL)
        elif goal_vel <= 0.0:
            self.set_motor_angle(0.0)

    def enable_torque_control(self):
        self.in_control_torque_mode = True
        self.previous_load_control_output = self.get_current_joint_angle()
        self.previous_load_control_error = 0.0

    def disable_torque_control(self):
        self.in_control_torque_mode = False

    def set_torque_cmd(self, torque_cmd):
        '''
        Bounds the given goal load and sets it as the goal
        '''
        self.torque_cmd = min(max(torque_cmd, 0.0), self.OVERLOAD_THRESHOLD)

    def control_torque(self, current_torque):
        current_error = self.torque_cmd - current_torque
        k = 16e-4 * 0.025  # Compensator gain - higher gain has faster response and is more unstable
                            # The 0.025 accounts for a 20Hz update rate
        output = self.previous_load_control_output + k * (current_error + self.previous_load_control_error)
        self.set_motor_angle(output)
        self.previous_load_control_output = output
        self.previous_load_control_error = current_error

    def loosen_if_overloaded(self, load):
        '''
        Takes the given load and checks against threshold, loosen motor if over
        '''
        if abs(load) > self.OVERLOAD_THRESHOLD:
            rospy.logwarn("Motor %s overloaded at %f, loosening" % (self.name, load))
            self.loosen()

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        self.set_motor_angle(self.motor_msg.joint_angle + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        self.set_motor_angle(self.motor_msg.joint_angle - loosen_angle)

    def receive_state_cb(self, data):
        self.motor_msg.joint_angle = data.joint_angle
        self.motor_msg.raw_angle = data.raw_angle
        self.motor_msg.velocity = data.velocity
        self.handle_motor_load(data.load)
        self.motor_msg.voltage = data.voltage
        self.motor_msg.temperature = data.temperature
        self.motor_msg.error_state = data.error_state

    def handle_motor_load(self, load):
        self.motor_msg.load = load
        if self.in_control_torque_mode:
            self.control_torque(self.motor_msg.load)
        self.loosen_if_overloaded(self.motor_msg.load)

