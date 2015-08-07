from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetSpeed

from motor import Motor


class ReflexSFMotor(Motor):
    def __init__(self, name):
        self.zero_point = rospy.get_param(self.name + '/zero_point')
        self.MOTOR_TO_JOINT_GEAR_RATIO = rospy.get_param(self.name + '/motor_to_joint_gear_ratio')
        self.motor_cmd_pub = rospy.Publisher(name + '/command', Float64, queue_size=10)
        self.set_speed_service = rospy.ServiceProxy(name + '/set_speed', SetSpeed)
        self.torque_enable_service = rospy.ServiceProxy(name + '/torque_enable', TorqueEnable)
        self.torque_enable_service(True)
        self.state_subscriber = rospy.Subscriber(name + '/state', JointState, self.receive_state_cb)

    def get_current_raw_motor_angle(self):
        return self.motor_msg.raw_angle

    def set_motor_angle(self, goal_pos):
        '''
        Bounds the given position command and sets it to the motor
        '''
        goal_pos *= self.MOTOR_TO_JOINT_GEAR_RATIO  # Goes from joint radians to motor radians
        self.motor_cmd_pub.publish(self.check_motor_angle_command(goal_pos))

    def check_motor_angle_command(self, angle_command):
        '''
        Returns given command if within the allowable range, returns bounded command if out of range
        '''
        angle_command = self.correct_motor_offset(angle_command)
        if self.MOTOR_TO_JOINT_INVERTED:
            bounded_command = max(min(angle_command, self.zero_point),
                                  self.zero_point - self.MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO)
        else:
            bounded_command = min(max(angle_command, self.zero_point),
                                  self.zero_point + self.MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO)
        return bounded_command

    def set_motor_speed(self, goal_speed):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.set_speed_service(self.check_motor_speed_command(goal_speed))

    def reset_motor_speed(self):
        '''
        Resets speed to default
        '''
        self.set_speed_service(self.DEFAULT_MOTOR_SPEED)

    def set_motor_velocity(self, goal_vel):
        '''
        Sets speed and commands finger in or out based on sign of velocity
        '''
        self.set_motor_speed(goal_vel)
        if goal_vel > 0.0:
            max_command = self.MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO
            self.motor_cmd_pub.publish(self.check_motor_angle_command(max_command))
        elif goal_vel < 0.0:
            self.motor_cmd_pub.publish(self.check_motor_angle_command(0.0))

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            tighten_angle *= -1
        self.set_raw_motor_angle(self.motor_msg.raw_angle + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            loosen_angle *= -1
        self.set_raw_motor_angle(self.motor_msg.raw_angle - loosen_angle)


    def receive_state_cb(self, data):
        # Calculate joint angle from motor angle
        if self.MOTOR_TO_JOINT_INVERTED:
            joint_angle = self.zero_point - data.current_pos
        else:
            joint_angle = data.current_pos - self.zero_point
        self.motor_msg.joint_angle = joint_angle / self.MOTOR_TO_JOINT_GEAR_RATIO
        self.motor_msg.raw_angle = data.current_pos
        self.motor_msg.velocity = data.velocity
        self.handle_motor_load(data.load)
        self.motor_msg.temperature = data.motor_temps[0]

    def handle_motor_load(self, load):
        load_filter = 0.25  # Rolling filter of noisy data
        if not self.MOTOR_TO_JOINT_INVERTED:
            load *= -1
        self.motor_msg.load = load_filter * load + (1 - load_filter) * self.motor_msg.load
        if self.in_control_force_mode:
            self.control_force(self.motor_msg.load, k=3.0*0.025)
        self.loosen_if_overloaded(self.motor_msg.load)

    def set_local_motor_zero_point(self):
        self.zero_point = self.motor_msg.raw_angle
        rospy.set_param(self.name + '/zero_point', self.motor_msg.raw_angle)

    def set_raw_motor_angle(self, goal_pos):
        self.motor_cmd_pub.publish(goal_pos)

    def correct_motor_offset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command

    def enable_torque(self):
        self.torque_enable_service(True)

    def disable_torque(self):
        self.torque_enable_service(False)
