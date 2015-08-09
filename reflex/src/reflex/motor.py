import rospy
from std_msgs.msg import Float64

import reflex_msgs.msg


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_takktile_f1 or /reflex_sf_f1
        '''
        self.name = name[1:]
        self.DEFAULT_MOTOR_SPEED = rospy.get_param(name + '/default_motor_speed')
        self.MAX_MOTOR_SPEED = rospy.get_param(name + '/max_motor_speed')
        self.MAX_MOTOR_TRAVEL = rospy.get_param(name + '/max_motor_travel')
        self.OVERLOAD_THRESHOLD = rospy.get_param(name + '/overload_threshold')
        self.motor_msg = reflex_msgs.msg.Motor()
        self.in_control_force_mode = False

    def get_current_joint_angle(self):
        return self.motor_msg.joint_angle

    def get_load(self):
        return self.motor_msg.load

    def get_motor_msg(self):
        return self.motor_msg

    def set_motor_angle(self, goal_pos):
        raise NotImplementedError

    def check_motor_angle_command(self, angle_command):
        raise NotImplementedError

    def set_motor_speed(self, goal_speed):
        raise NotImplementedError

    def reset_motor_speed(self):
        raise NotImplementedError

    def set_motor_velocity(self, goal_vel):
        raise NotImplementedError

    def tighten(self, tighten_angle=0.05):
        raise NotImplementedError

    def loosen(self, loosen_angle=0.05):
        raise NotImplementedError

    def receive_state_cb(self, data):
        raise NotImplementedError

    def handle_motor_load(self, load):
        raise NotImplementedError

    def check_motor_speed_command(self, goal_speed):
        '''
        Returns absolute of given command if within the allowable range,
        returns bounded command if out of range. Always returns positive
        '''
        bounded_command = min(abs(goal_speed), self.MAX_MOTOR_SPEED)
        return bounded_command

    def enable_force_control(self):
        self.in_control_force_mode = True
        self.previous_load_control_output = self.get_current_joint_angle()
        self.previous_load_control_error = 0.0

    def disable_force_control(self):
        self.in_control_force_mode = False

    def set_force_cmd(self, force_cmd):
        '''
        Bounds the given goal load and sets it as the goal
        '''
        self.force_cmd = min(max(force_cmd, 0.0), self.OVERLOAD_THRESHOLD)

    def control_force(self, current_force, k):
        '''
        Uses discrete integral control to try and maintain goal force
        k is Compensator gain - higher gain has faster response and is more unstable
        '''
        current_error = self.force_cmd - current_force
        output = self.previous_load_control_output + k * (current_error + self.previous_load_control_error)
        self.set_motor_angle(output)
        self.previous_load_control_output = output
        self.previous_load_control_error = current_error

    def loosen_if_overloaded(self, load):
        '''
        Takes the given load and checks against threshold, loosen motor if over
        '''
        if abs(load) > self.OVERLOAD_THRESHOLD:
            rospy.logwarn("Motor %s overloaded at %f, loosening", self.name, load)
            self.loosen()
